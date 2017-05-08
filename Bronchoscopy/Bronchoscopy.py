import os
import unittest
from __main__ import vtk, qt, ctk, slicer
import numpy
import numpy.linalg
from vtk.util.numpy_support import vtk_to_numpy
import csv
import math
import time
import SimpleITK as sitk

#
# Bronchoscopy
#

class Bronchoscopy:
  def __init__(self, parent):
    parent.title = "Bronchoscopy" # TODO make this more human readable by adding spaces
    parent.categories = ["Endoscopy"]
    parent.dependencies = []
    parent.contributors = ["Pietro Nardelli (University College Cork)"] 
    parent.helpText = """
    Scripted module bundled in an extension for centerline extraction and virtual navigation within a 3D airway model.
    """
    parent.acknowledgementText = """
    This file was originally developed by Pietro Nardelli and Alberto Corvo', University College Cork.
""" # replace with organization, grant and thanks.
    self.parent = parent

#
# qBronchoscopyWidget
#
class BronchoscopyWidget:
  def __init__(self, parent = None):
    if not parent:
      self.parent = slicer.qMRMLWidget()
      self.parent.setLayout(qt.QVBoxLayout())
      self.parent.setMRMLScene(slicer.mrmlScene)
    else:
      self.parent = parent


    self.pendingUpdate = False
    self.updatingFiducials = False
    self.observeTags = []

    self.addNewPathPoints = False
    
    self.layout = self.parent.layout()
    self.cameraNode = None
    self.cameraNodeObserverTag = None
    self.cameraObserverTag = None

    self.centerlinePointsList = []
    self.centerline = None
    self.fiducialNode = None
    self.uploadedCenterlineModel = None

    self.pathCreated = 0

    self.pathModelNamesList = []

    self.bifurcationPointsList = []

    #
    # Sensor Tracking Variables
    #
    self.sensorTimer = qt.QTimer()
    self.sensorTimer.setInterval(1)
    self.sensorTimer.connect('timeout()', self.ReadPosition)

    self.time = time.time()

    self.checkStreamingTimer = qt.QTimer()
    self.checkStreamingTimer.setInterval(1)
    self.checkStreamingTimer.connect('timeout()', self.showVideoStreaming)

    self.previousMatrixSigns = []
    
    self.flipCompensationTransform = None
    self.probeCalibrationTransform = None
    self.centerlineCompensationTransform = None
    self.cameraForNavigation = None
    self.cNode = None
    self.lastFPBeforeStoppingTracking = [0,0,0]
    self.lastPosBeforeStoppingTracking = [0,0,0]
    self.lastViewUp = [0,0,0]

    self.thirdCamera = None
    self.thirdCameraInitialized = 0

    self.length = 1000000000

    self.probeToTrackerTransformNode = None
    self.videoStreamingNode = None

    self.customLayoutId = 501
    self.three3DViewsLayoutId = 502    

    self.layoutManager = slicer.app.layoutManager()
    
    self.setThree3Dviews()
    self.setLayout()

    viewNode1 = slicer.util.getNode('vtkMRMLViewNode1')
    viewNode2 = slicer.util.getNode('vtkMRMLViewNode2')

    self.firstThreeDView = self.layoutManager.viewWidget(viewNode1).threeDView()
    self.secondThreeDView = self.layoutManager.viewWidget( viewNode2 ).threeDView()
    self.thirdThreeDView = None

    self.updateGUI()

    if not parent:
      self.setup()
      self.parent.show()
      self.updateGUI()

  def setLayout(self):
    customLayout = ("<layout type=\"vertical\" split=\"false\" >"
                    " <item>"
                    "  <layout type=\"horizontal\">"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"RealView\">"
                    "     <property name=\"orientation\" action=\"default\">Axial</property>"
                    "     <property name=\"viewlabel\" action=\"default\">RV</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#8C8C8C</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLViewNode\" singletontag=\"1\">"
                    "     <property name=\"viewlabel\" action=\"default\">1</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLViewNode\" singletontag=\"2\" type=\"secondary\">"
                    "     <property name=\"viewlabel\" action=\"default\">2</property>"
                    "    </view>"
                    "   </item>"
                    "  </layout>"
                    " </item>"
                    " <item>"
                    "  <layout type=\"horizontal\">"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"Red\">"
                    "     <property name=\"orientation\" action=\"default\">Axial</property>"
                    "     <property name=\"viewlabel\" action=\"default\">R</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#F34A33</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"Yellow\">"
                    "     <property name=\"orientation\" action=\"default\">Sagittal</property>"
                    "     <property name=\"viewlabel\" action=\"default\">Y</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#EDD54C</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"Green\">"
                    "     <property name=\"orientation\" action=\"default\">Coronal</property>"
                    "     <property name=\"viewlabel\" action=\"default\">G</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#6EB04B</property>"
                    "    </view>"
                    "   </item>"
                    "  </layout>"
                    " </item>"
                    "</layout>")
    self.layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(self.customLayoutId, customLayout)
    self.layoutManager.setLayout(self.customLayoutId)

  def setThree3Dviews(self):
    layout = ("<layout type=\"vertical\" split=\"true\" >"
                    " <item>"
                    "  <layout type=\"horizontal\">"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"RealView\">"
                    "     <property name=\"orientation\" action=\"default\">Axial</property>"
                    "     <property name=\"viewlabel\" action=\"default\">RV</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#8C8C8C</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLViewNode\" singletontag=\"1\">"
                    "     <property name=\"First3DView\" action=\"default\">1</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLViewNode\" singletontag=\"2\" type=\"secondary\">"
                    "     <property name=\"Second3DView\" action=\"default\">2</property>"
                    "    </view>"
                    "   </item>"
                    "  </layout>"
                    " </item>"
                    " <item>"
                    "  <layout type=\"horizontal\">"
                    "   <item>"
                    "    <view class=\"vtkMRMLViewNode\" singletontag=\"3\" type=\"endoscopy\">"
                    "     <property name=\"Third3DView\" action=\"default\">3</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"Red\">"
                    "     <property name=\"orientation\" action=\"default\">Axial</property>"
                    "     <property name=\"viewlabel\" action=\"default\">R</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#F34A33</property>"
                    "    </view>"
                    "   </item>"
                    "   <item>"
                    "    <view class=\"vtkMRMLSliceNode\" singletontag=\"Green\">"
                    "     <property name=\"orientation\" action=\"default\">Coronal</property>"
                    "     <property name=\"viewlabel\" action=\"default\">G</property>"
                    "     <property name=\"viewcolor\" action=\"default\">#6EB04B</property>"
                    "    </view>"
                    "   </item>"
                    "  </layout>"
                    " </item>"
                    "</layout>")
    self.layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(self.three3DViewsLayoutId, layout)
    #self.layoutManager.setLayout(self.three3DViewsLayoutId)

  def cleanup(self):
    pass

  def updateGUI(self):
    if(self.thirdThreeDView):
      self.layoutManager.setLayout(19)
      self.layoutManager.setLayout(self.three3DViewsLayoutId)
      self.thirdThreeDView.resetFocalPoint()
      self.thirdThreeDView.lookFromViewAxis(ctk.ctkAxesWidget().Anterior)
    else:
      self.layoutManager.setLayout(19)
      self.layoutManager.setLayout(self.customLayoutId)

    self.firstThreeDView.resetFocalPoint()
    self.firstThreeDView.lookFromViewAxis(ctk.ctkAxesWidget().Anterior)

    self.secondThreeDView.resetFocalPoint()
    self.secondThreeDView.lookFromViewAxis(ctk.ctkAxesWidget().Anterior)  

    red_logic = slicer.app.layoutManager().sliceWidget("Red").sliceLogic()
    red_cn = red_logic.GetSliceCompositeNode()
    volumeID = red_cn.GetBackgroundVolumeID()
    if volumeID:
      self.improveCTContrast(volumeID)
      
  def improveCTContrast(self, volID):
    volume = slicer.util.getNode(volID)
    displayNode = volume.GetDisplayNode()
    displayNode.SetAutoWindowLevel(0)
    displayNode.SetWindowLevel(1400,-500)

  def setup(self):
    #
    # Parameters Area
    #
    parametersCollapsibleButton = ctk.ctkCollapsibleButton()
    parametersCollapsibleButton.text = "Centerline Extraction Area"
    self.layout.addWidget(parametersCollapsibleButton)
    self.layout.setSpacing(20)
    # Layout within the dummy collapsible button
    IOFormLayout = qt.QFormLayout(parametersCollapsibleButton)

    ###################################################################################
    ##############################  3D Model Selector  ################################
    ###################################################################################
    self.inputSelector = slicer.qMRMLNodeComboBox()
    self.inputSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
    self.inputSelector.selectNodeUponCreation = True
    self.inputSelector.addEnabled = False
    self.inputSelector.removeEnabled = True
    self.inputSelector.noneEnabled = False
    self.inputSelector.showHidden = False
    self.inputSelector.showChildNodeTypes = False
    self.inputSelector.setMRMLScene( slicer.mrmlScene )
    self.inputSelector.setToolTip( "Pick the 3D input model to the algorithm." )
    IOFormLayout.addRow("3D Airway Model: ", self.inputSelector)

    if( self.inputSelector.currentNode() ):
      inputVolume = self.inputSelector.currentNode()
      modelDisplayNode = inputVolume.GetDisplayNode()
      modelDisplayNode.SetColor(1.0, 0.8, 0.7)
      modelDisplayNode.SetFrontfaceCulling(1)
      modelDisplayNode.SetBackfaceCulling(0)
      modelDisplayNode.SetAmbient(0.08)
      modelDisplayNode.SetDiffuse(0.90)
      modelDisplayNode.SetSpecular(0.17)

    ###################################################################################
    ###############################  Label Selector  ##################################
    ###################################################################################
    self.labelSelector = slicer.qMRMLNodeComboBox()
    self.labelSelector.nodeTypes = ( ("vtkMRMLLabelMapVolumeNode"), "" )
    # self.labelSelector.addAttribute( "vtkMRMLScalarVolumeNode", "LabelMap", 1 )
    self.labelSelector.selectNodeUponCreation = True
    self.labelSelector.addEnabled = False
    self.labelSelector.removeEnabled = True
    self.labelSelector.noneEnabled = False
    self.labelSelector.showHidden = False
    self.labelSelector.showChildNodeTypes = False
    self.labelSelector.setMRMLScene( slicer.mrmlScene )
    self.labelSelector.setToolTip( "Pick the 3D input model to the algorithm." )
    IOFormLayout.addRow("Airway Label: ", self.labelSelector)

    ####################################################################################
    #### Optional Collapsible Button To Select An Uploaded Centerline Fiducials List ###
    ####################################################################################

    boxLayout = qt.QVBoxLayout()

    self.fiducialsCollapsibleButton = ctk.ctkCollapsibleButton()
    self.fiducialsCollapsibleButton.text = "Centerline Fiducials List/ Centerline Model"
    self.fiducialsCollapsibleButton.setChecked(False)
    self.fiducialsCollapsibleButton.setFixedSize(400,100)
    self.fiducialsCollapsibleButton.enabled = True
    boxLayout.addWidget(self.fiducialsCollapsibleButton, 0, 4)
    fiducialFormLayout = qt.QFormLayout(self.fiducialsCollapsibleButton)

    self.fiducialListSelector = slicer.qMRMLNodeComboBox()
    self.fiducialListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.fiducialListSelector.selectNodeUponCreation = True
    self.fiducialListSelector.addEnabled = False
    self.fiducialListSelector.removeEnabled = True
    self.fiducialListSelector.noneEnabled = True
    self.fiducialListSelector.showHidden = False
    self.fiducialListSelector.showChildNodeTypes = False
    self.fiducialListSelector.setMRMLScene( slicer.mrmlScene )
    self.fiducialListSelector.setToolTip( "Select uploaded centerline fiducial list" )
    fiducialFormLayout.addRow("Centerline Fiducials List: ", self.fiducialListSelector)

    self.centerlineModelSelector = slicer.qMRMLNodeComboBox()
    self.centerlineModelSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
    self.centerlineModelSelector.selectNodeUponCreation = True
    self.centerlineModelSelector.addEnabled = False
    self.centerlineModelSelector.removeEnabled = True
    self.centerlineModelSelector.noneEnabled = True
    self.centerlineModelSelector.showHidden = False
    self.centerlineModelSelector.showChildNodeTypes = False
    self.centerlineModelSelector.setMRMLScene( slicer.mrmlScene )
    self.centerlineModelSelector.setToolTip( "Select uploaded centerline model" )
    fiducialFormLayout.addRow("Centerline Model: ", self.centerlineModelSelector)

    ########################################################################################################
    #### Optional Push Button To Create A List Of Fiducial Starting From The Extracted Centerline Points ###
    ########################################################################################################

    self.CreateFiducialListButton = qt.QPushButton("Create and Save a Fiducial List From Centerline")
    self.CreateFiducialListButton.toolTip = "Create a list of fiducial points starting from the extracted centerline of the 3D model."
    self.CreateFiducialListButton.setFixedSize(250,25)

    if self.centerlinePointsList != []:
      self.CreateFiducialListButton.enabled = True
    else:
      self.CreateFiducialListButton.enabled = False
    box = qt.QVBoxLayout()
    fiducialFormLayout.addRow(box)

    ###################################################################################
    #########################  Extract Centerline Button  #############################
    ###################################################################################
    self.ExtractCenterlineButton = qt.QPushButton("Extract Centerline")
    self.ExtractCenterlineButton.toolTip = "Run the algorithm to extract centerline of the model on which fiducials will be placed. Fiducials are necessary to compensate for possible registration issues."
    self.ExtractCenterlineButton.setFixedSize(200,50)
    if self.inputSelector.currentNode() and self.labelSelector.currentNode():
        self.ExtractCenterlineButton.enabled = True
        self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(175,255,253)")
    else:
        self.ExtractCenterlineButton.enabled = False
        self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(255,255,255)")

    IOFormLayout.addRow(boxLayout)
    boxLayout.addWidget(self.ExtractCenterlineButton,0,4)
    boxLayout.addWidget(self.CreateFiducialListButton,0,4)

    ####################################################################################
    ############  Create Path Towards An ROI Section (Procedure Planning)  #############
    ####################################################################################
    self.pathCreationCollapsibleButton = ctk.ctkCollapsibleButton()
    self.pathCreationCollapsibleButton.text = "Procedure Planning"
    self.pathCreationCollapsibleButton.setChecked(True)
    self.layout.addWidget(self.pathCreationCollapsibleButton)
    pathCreationFormLayout = qt.QFormLayout(self.pathCreationCollapsibleButton)

    # Button To Create A Fiducial List Containing All The Points On The ROIs
    self.createROIFiducialsButton = qt.QPushButton("Add New ROI Point(s)")
    self.createROIFiducialsButton.toolTip = "Add new ROI point(s)."
    self.createROIFiducialsButton.setFixedSize(160,35)

    pointsButtonsBox = qt.QHBoxLayout()
    pathCreationFormLayout.addRow(pointsButtonsBox)

    #ROIBox.addWidget(self.pointsListSelector, 0, 4)
    pointsButtonsBox.addWidget(self.createROIFiducialsButton, 0, 4)

    # Button To Create A Fiducial List Containing The Points On The Labels Closest To The ROIs
    self.createLabelsFiducialsButton = qt.QPushButton("Add New Label Point(s)")
    self.createLabelsFiducialsButton.toolTip = "Add point(s) on the closest labels to the ROIs."
    self.createLabelsFiducialsButton.setFixedSize(160,35)
    self.createLabelsFiducialsButton.enabled = False
    pointsButtonsBox.addWidget(self.createLabelsFiducialsButton, 0, 4)

    # Combobox listing all the ROIs points
    self.ROIsPoints = qt.QComboBox()
    self.ROIsPoints.setFixedWidth(180)

    # Button to create new path points
    self.createNewPathPointsButton = qt.QPushButton("Add New Path Point(s)")
    self.createNewPathPointsButton.toolTip = "Add new path point(s) to improve path creation."
    self.createNewPathPointsButton.setFixedSize(160,35)
    self.createNewPathPointsButton.enabled = False

    ROIPointSelectionBox = qt.QHBoxLayout()
    #pathCreationFormLayout.addRow("New Path Points List: ",)
    pathCreationFormLayout.addRow(ROIPointSelectionBox)

    ROIPointSelectionBox.addWidget(self.ROIsPoints, 0, 4)
    ROIPointSelectionBox.addWidget(self.createNewPathPointsButton, 0, 4)
    
    # 
    # Buttons to enlarge views upon request
    #
    layoutGroupBox = qt.QFrame()
    layoutGroupBox.setLayout(qt.QVBoxLayout())
    layoutGroupBox.setFixedHeight(86)
    pathCreationFormLayout.addRow(layoutGroupBox) 

    buttonGroupBox = qt.QFrame()
    buttonGroupBox.setLayout(qt.QHBoxLayout())
    layoutGroupBox.layout().addWidget(buttonGroupBox)

    #
    # Default Layout Button
    #
    self.defaultButton = qt.QPushButton("Def")
    self.defaultButton.toolTip = "Default layout button."
    self.defaultButton.enabled = True
    self.defaultButton.setFixedSize(40,40)
    buttonGroupBox.layout().addWidget(self.defaultButton)
   
    #
    # Red Slice Button
    #
    self.redViewButton = qt.QPushButton()
    self.redViewButton.toolTip = "Red slice only."
    self.redViewButton.enabled = True
    self.redViewButton.setFixedSize(40,40)
    redIcon = qt.QIcon(":/Icons/LayoutOneUpRedSliceView.png")
    self.redViewButton.setIcon(redIcon)
    buttonGroupBox.layout().addWidget(self.redViewButton)

    #
    # Yellow Slice Button
    #
    self.yellowViewButton = qt.QPushButton()
    self.yellowViewButton.toolTip = "Yellow slice only."
    self.yellowViewButton.enabled = True
    self.yellowViewButton.setFixedSize(40,40)
    yellowIcon = qt.QIcon(":/Icons/LayoutOneUpYellowSliceView.png")
    self.yellowViewButton.setIcon(yellowIcon)
    buttonGroupBox.layout().addWidget(self.yellowViewButton)
    
    #
    # Green Slice Button
    #
    self.greenViewButton = qt.QPushButton()
    self.greenViewButton.toolTip = "Yellow slice only."
    self.greenViewButton.enabled = True
    self.greenViewButton.setFixedSize(40,40)   
    greenIcon = qt.QIcon(":/Icons/LayoutOneUpGreenSliceView.png")
    self.greenViewButton.setIcon(greenIcon) 
    buttonGroupBox.layout().addWidget(self.greenViewButton)

    ###################################################################################
    #############################  Path Creation Button  ############################## 
    ###################################################################################
    
    self.PathCreationButton = qt.QPushButton("Create Path(s)")
    self.PathCreationButton.toolTip = "Run the algorithm to create the path between the specified points."
    self.PathCreationButton.setFixedSize(300,50)
    self.PathCreationButton.enabled = False

    '''if self.inputSelector.currentNode() and self.pointsListSelector.currentNode() and self.centerlinePointsList != []:
        self.PathCreationButton.enabled = True
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
    else:
        self.PathCreationButton.enabled = False
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")'''

    bLayout = qt.QVBoxLayout()

    pathCreationFormLayout.addRow(bLayout)
    bLayout.addWidget(self.PathCreationButton,0,4)

    #################################################################################
    ################ Path Visualization And Distance To Target Info #################
    #################################################################################

    self.pathInfoCollapsibleButton = ctk.ctkCollapsibleButton()
    self.pathInfoCollapsibleButton.text = "Path Visualization and Distance To Target Information"
    self.pathInfoCollapsibleButton.setChecked(True)
    self.pathInfoCollapsibleButton.enabled = True
    self.layout.addWidget(self.pathInfoCollapsibleButton)
    pathInfoFormLayout = qt.QFormLayout(self.pathInfoCollapsibleButton)

    self.pathModelSelector = slicer.qMRMLNodeComboBox()
    self.pathModelSelector.nodeTypes = ( ("vtkMRMLModelNode"), "" )
    self.pathModelSelector.selectNodeUponCreation = True
    self.pathModelSelector.addEnabled = False
    self.pathModelSelector.removeEnabled = True
    self.pathModelSelector.noneEnabled = True
    self.pathModelSelector.showHidden = False
    self.pathModelSelector.showChildNodeTypes = False
    self.pathModelSelector.setMRMLScene( slicer.mrmlScene )
    self.pathModelSelector.setToolTip( "Pick the 3D path model to visualize." )
    pathInfoFormLayout.addRow("Path Model: ", self.pathModelSelector)

    self.pathLength = qt.QLineEdit()
    self.pathLength.setReadOnly(True)

    self.distanceToTarget = qt.QLineEdit()
    self.distanceToTarget.setReadOnly(True)

    pathInfoFormLayout.addRow("Path Length:", self.pathLength)
    pathInfoFormLayout.addRow("Distance To Target: ", self.distanceToTarget)

    ###################################################################################
    ##########################  Fiducial Registration Area ############################
    ###################################################################################

    registrationCollapsibleButton = ctk.ctkCollapsibleButton()
    registrationCollapsibleButton.text = "Fiducial Registration Area"
    self.layout.addWidget(registrationCollapsibleButton)
    self.layout.setSpacing(20)
    # Layout within the dummy collapsible button
    registrationFormLayout = qt.QFormLayout(registrationCollapsibleButton)

    regBox = qt.QHBoxLayout()
    registrationFormLayout.addRow(regBox)
    
    self.registrationSelector = slicer.qMRMLNodeComboBox()
    self.registrationSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.registrationSelector.selectNodeUponCreation = True
    self.registrationSelector.addEnabled = False
    self.registrationSelector.baseName = 'RegistrationPoints'
    self.registrationSelector.removeEnabled = True
    self.registrationSelector.noneEnabled = True
    self.registrationSelector.showHidden = False
    self.registrationSelector.showChildNodeTypes = False
    self.registrationSelector.setMRMLScene( slicer.mrmlScene )
    self.registrationSelector.setToolTip( "Select registration fiducial points" )
    self.registrationSelector.setFixedWidth(200)
    #registrationFormLayout.addRow("Registration Fiducials List: ", self.registrationSelector)

    self.createRegistrationFiducialsButton = qt.QPushButton("Add Registration Point(s)")
    self.createRegistrationFiducialsButton.toolTip = "Create fiducial list for the registration."
    self.createRegistrationFiducialsButton.setFixedSize(200,35)
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,246,142)")

    regBox.addWidget(self.registrationSelector)
    regBox.addWidget(self.createRegistrationFiducialsButton)

    self.folderPathSelection = qt.QLineEdit()
    self.folderPathSelection.setReadOnly(True)
    self.folderPathSelection.setFixedWidth(200)

    selectionBox = qt.QHBoxLayout()
    registrationFormLayout.addRow(selectionBox)

    self.selectFolderButton = qt.QPushButton("Select Folder")
    self.selectFolderButton.toolTip = "Select folder where to save the txt file containing the fiducial points."
    self.selectFolderButton.setFixedSize(200,35)
    self.selectFolderButton.setStyleSheet("background-color: rgb(255,246,142)")

    selectionBox.addWidget(self.folderPathSelection)
    selectionBox.addWidget(self.selectFolderButton)

    self.RegFidListButton = qt.QPushButton("Save Registration Points")
    self.RegFidListButton.toolTip = "Create a list of fiducial points starting from the extracted centerline of the 3D model."
    self.RegFidListButton.setFixedSize(150,45)

    if( self.registrationSelector.currentNode() and self.folderPathSelection.text):
      self.RegFidListButton.enabled = True
    else:
      self.RegFidListButton.enabled = False

    registrationBox = qt.QVBoxLayout()
    registrationFormLayout.addRow(registrationBox)
    registrationBox.addWidget(self.RegFidListButton,0,4)

    #############################################################################################
    ###########################  Sensor Tracker Collapsible Button  #############################
    #############################################################################################

    trackerCollapsibleButton = ctk.ctkCollapsibleButton()
    trackerCollapsibleButton.text = "Probe Tracking"
    self.layout.addWidget(trackerCollapsibleButton)
    self.layout.setSpacing(20)
    trackerFormLayout = qt.QFormLayout(trackerCollapsibleButton)

    trackerButtonLayout = qt.QHBoxLayout()

    ##############################################################################################
    ###############################  Matlab/Probe Track Button  ##################################
    ##############################################################################################
    self.ProbeTrackButton = qt.QPushButton("Track Sensor")
    self.ProbeTrackButton.toolTip = "Track sensor output."
    self.ProbeTrackButton.setFixedSize(250,60)
    #self.ProbeTrackButton.setFixedHeight(40)
    self.ProbeTrackButton.checkable = True
   
    trackerButtonLayout.addWidget(self.ProbeTrackButton, 0, 4)

    trackerFormLayout.addRow(trackerButtonLayout)

    # Enable ProbeTracKButton
    if self.centerlinePointsList != []:
      self.ProbeTrackButton.enabled = True
    else:
      self.ProbeTrackButton.enabled = False

    ############################################################################################
    ##################################  Three 3D Views Button  #####################################
    ############################################################################################

    self.newLayoutImageButton = qt.QPushButton("Add Third 3D View")

    self.newLayoutImageButton.toolTip = "Change to layout with a third 3D view to help navigation"
    self.newLayoutImageButton.setFixedSize(250,35)
    self.newLayoutImageButton.enabled = True
    self.newLayoutImageButton.checkable = True

    newLayoutBox = qt.QHBoxLayout()
    
    newLayoutBox.addWidget(self.newLayoutImageButton, 0, 4)

    trackerFormLayout.addRow(newLayoutBox)

    ############################################################################################
    ##################################  Flip Image Button  #####################################
    ############################################################################################

    self.FlipImageButton = qt.QPushButton("180 Flip")

    self.FlipImageButton.toolTip = "Compensate for possible 180 degree camera flipping."
    self.FlipImageButton.setFixedSize(100,35)
    self.FlipImageButton.enabled = False

    flippingBox = qt.QHBoxLayout()
    
    flippingBox.addWidget(self.FlipImageButton, 0, 4)

    trackerFormLayout.addRow(flippingBox)


    #####################################################################################
    ################################ Video Streaming ####################################
    #####################################################################################
    videoStreamCollapsibleButton = ctk.ctkCollapsibleButton()
    videoStreamCollapsibleButton.text = "Video Streaming Section"
    self.layout.addWidget(videoStreamCollapsibleButton)
    self.layout.setSpacing(20)
    videoStreamingFormLayout = qt.QFormLayout(videoStreamCollapsibleButton)

    videoStreamingSelectionBox = qt.QHBoxLayout()
    videoStreamingFormLayout.addRow(videoStreamingSelectionBox)

    self.VideoRegistrationButton = qt.QPushButton("Start Video Streaming")
    self.VideoRegistrationButton.toolTip = "Stream the real video within the lung"
    self.VideoRegistrationButton.setFixedSize(270,50)
    self.VideoRegistrationButton.enabled = True
    self.VideoRegistrationButton.checkable = True
    
    VSButtonBox = qt.QVBoxLayout()
    videoStreamingFormLayout.addRow(VSButtonBox)

    VSButtonBox.addWidget(self.VideoRegistrationButton, 0, 4)

    ########################################################################################
    ################################ Image Registration ####################################
    ########################################################################################
    self.ImageRegistrationButton = qt.QPushButton("Start Image Registration")
    self.ImageRegistrationButton.toolTip = "Start registration between real and virtual images."
    self.ImageRegistrationButton.setFixedSize(200,40)
    self.ImageRegistrationButton.enabled = False
    self.ImageRegistrationButton.checkable = True
    self.ImageRegistrationButton.hide()
    
    VSButtonBox.addWidget(self.ImageRegistrationButton, 0, 4)

    ########################################################################################
    ################################ Create Connections ####################################
    ########################################################################################

    self.registrationSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.selectFolderButton.connect('clicked(bool)', self.onSelectFolderButton)
    self.createRegistrationFiducialsButton.connect('clicked(bool)', self.onCreateRegFidList)
    self.RegFidListButton.connect('clicked(bool)', self.onSaveRegistrationPoints)

    self.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.labelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.ExtractCenterlineButton.connect('clicked(bool)', self.onExtractCenterlineButton)
    self.fiducialListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.centerlineModelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.CreateFiducialListButton.connect('clicked(bool)',self.onCreateAndSaveFiducialList)

    #self.pointsListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.createROIFiducialsButton.connect('clicked(bool)', self.onCreateROIFiducialsList)
    self.createLabelsFiducialsButton.connect('clicked(bool)', self.onCreateLabelsFiducialsList)
    self.ROIsPoints.connect('currentIndexChanged(int)', self.showSelectedROI)

    self.defaultButton.connect('clicked()', self.onDefaultLayoutButton)
    self.redViewButton.connect('clicked()', self.onRedViewButton)
    self.yellowViewButton.connect('clicked()', self.onYellowViewButton)
    self.greenViewButton.connect('clicked()', self.onGreenViewButton)

    self.createNewPathPointsButton.connect('clicked(bool)', self.startAddingNewPathPoints)
    
    self.PathCreationButton.connect('clicked(bool)', self.onPathCreationButton)
    self.pathModelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onPathSelect)

    self.ProbeTrackButton.connect('toggled(bool)', self.onProbeTrackButtonToggled)
    self.newLayoutImageButton.connect('toggled(bool)', self.onChangeLayoutButtonToggled)
    self.FlipImageButton.connect('clicked(bool)', self.onFlipImageButton)

    self.ImageRegistrationButton.connect('toggled(bool)',self.onStartImageRegistrationButtonPressed)

    self.VideoRegistrationButton.connect('toggled(bool)',self.startVideoStreaming)
    
    #
    # Add Vertical Spacer
    #
    self.layout.addStretch(1)
    
    #
    # Update the 3D Views
    #
    self.updateGUI()

    #
    # Update ROIs List
    #
    self.updateList()

  def updateList(self):
    '''Observe the mrml scene for changes that we wish to respond to.'''
    tag = slicer.mrmlScene.AddObserver(slicer.mrmlScene.EndCloseEvent, self.clearROIsComboBox)
    tag = slicer.mrmlScene.AddObserver(slicer.mrmlScene.NodeAddedEvent, self.requestNodeAddedUpdate)
    self.observeTags.append((slicer.mrmlScene,tag))

  def clearROIsComboBox(self):
    self.ROIsPoints.clear()    
  
  def onSelect(self):
    self.updateGUI()

    if self.inputSelector.currentNode() and self.labelSelector.currentNode():
      inputVolume = self.inputSelector.currentNode()
      modelDisplayNode = inputVolume.GetDisplayNode()
      modelDisplayNode.SetColor(1.0, 0.8, 0.7)
      modelDisplayNode.SetFrontfaceCulling(1)
      modelDisplayNode.SetBackfaceCulling(0)

      self.ExtractCenterlineButton.enabled = True
      self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(175,255,253)")
   
      if self.centerlinePointsList != []:
        self.ProbeTrackButton.enabled = True
      else:
        self.ProbeTrackButton.enabled = False
        self.FlipImageButton.enabled = False
        #self.ImageRegistrationButton.enabled = False
    else:
      self.ExtractCenterlineButton.enabled = False
      self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.PathCreationButton.enabled = False
      self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.ProbeTrackButton.enabled = False
      self.FlipImageButton.enabled = False
      #self.ImageRegistrationButton.enabled = False

    if self.registrationSelector.currentNode() and self.folderPathSelection.text:

      self.RegFidListButton.enabled = True
    else:
      self.RegFidListButton.enabled = False

    ROIfids = slicer.util.getNode('ROIFiducials')
    if self.ROIsPoints.currentIndex == -1 and ROIfids:
      self.fillComboBox(ROIfids)

    if self.ROIsPoints.currentIndex >= 0: 
      self.createLabelsFiducialsButton.enabled = True
      self.createNewPathPointsButton.enabled = True

      if self.inputSelector.currentNode() and self.centerlinePointsList != []:
         self.PathCreationButton.enabled = True
         self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
      else:
         self.PathCreationButton.enabled = False
         self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")
    else:
       self.createLabelsFiducialsButton.enabled = False
       self.createNewPathPointsButton.enabled = False
       self.PathCreationButton.enabled = False
       self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")

    if self.centerlinePointsList != []:
      self.CreateFiducialListButton.enabled = True

  def fillComboBox(self, ROIfiducials):
    if ROIfiducials.GetNumberOfFiducials() > 0:
      for i in xrange(ROIfiducials.GetNumberOfFiducials()):
        self.ROIsPoints.addItem(ROIfiducials.GetNthFiducialLabel(i))
        self.ROIsPoints.setCurrentIndex(self.ROIsPoints.count-1)

  def onPathCreationSelection(self):

    if self.ROIsPoints.currentIndex >= 0:
 
      self.createLabelsFiducialsButton.enabled = True
      self.createNewPathPointsButton.enabled = True

      if self.inputSelector.currentNode() and self.centerlinePointsList != []:
         self.PathCreationButton.enabled = True
         self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
      else:
         self.PathCreationButton.enabled = False
         self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")
    else:
       self.createLabelsFiducialsButton.enabled = False
       self.createNewPathPointsButton.enabled = False
       self.PathCreationButton.enabled = False
       self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")

  def disableButtonsAndSelectors(self):

    self.selectFolderButton.enabled = False
    self.selectFolderButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createRegistrationFiducialsButton.enabled = False
    self.RegFidListButton.enabled = False
    self.ExtractCenterlineButton.enabled = False
    self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(255,255,255)")    
    self.CreateFiducialListButton.enabled = False
    self.PathCreationButton.enabled = False
    self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")    
    self.ProbeTrackButton.enabled = False
    self.FlipImageButton.enabled = False
    #self.ImageRegistrationButton.enabled = False

    self.inputSelector.enabled = False
    self.labelSelector.enabled = False
    self.fiducialListSelector.enabled = False
    self.centerlineModelSelector.enabled = False
    #self.pointsListSelector.enabled = False
    self.registrationSelector.enabled = False

  def enableSelectors(self):

    self.selectFolderButton.enabled = True
    self.selectFolderButton.setStyleSheet("background-color: rgb(255,246,142)")

    self.createRegistrationFiducialsButton.enabled = True
    #self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,246,142)")

    self.inputSelector.enabled = True
    self.labelSelector.enabled = True
    self.fiducialListSelector.enabled = True
    self.centerlineModelSelector.enabled = True
    self.registrationSelector.enabled = True

##################################################################################################
############################## REGISTRATION FIDUCIALS SAVING ##################################### 
##################################################################################################
# Copy fiducials used for the registration within a txt file to be saved in the Matlab folder

  def onCreateRegFidList(self):

    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.Reset()

    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,99,71)")
    self.createROIFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createLabelsFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createNewPathPointsButton.setStyleSheet("background-color: rgb(255,255,255)")

    self.disableButtonsAndSelectors()
    markupsList = slicer.util.getNode('RegistrationMarker')
    if markupsList == None:
      markupsList = slicer.vtkMRMLMarkupsFiducialNode()
      markupsList.SetName('RegistrationMarker')
      slicer.mrmlScene.AddNode(markupsList)
    
    displayNode = markupsList.GetDisplayNode()
    displayNode.SetSelectedColor(0.0,0.0,1.0)

    self.registrationSelector.setCurrentNodeID(markupsList.GetID())
    markupLogic = slicer.modules.markups.logic()
    markupLogic.SetActiveListID(markupsList)   

    self.enableSelectors()
    self.onSelect()

  def onSaveRegistrationPoints(self):
    
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.disableButtonsAndSelectors()
   
    regFidListNode = self.registrationSelector.currentNode()
    point = [0,0,0]
    pointsList = []
    for i in xrange(regFidListNode.GetNumberOfFiducials()):
      regFidListNode.GetNthFiducialPosition(i,point)
      p = [point[0],point[1],point[2]]
      pointsList.append(p)

    localDirectory = self.folderPathSelection.text + "/F.csv"

    with open(localDirectory, "wb") as f:
      writer = csv.writer(f, delimiter=' ')
      writer.writerows(pointsList) 

    FList = slicer.mrmlScene.GetNodesByName('F')
    AirwayFiducialList = slicer.mrmlScene.GetNodesByName('AirwayFiducial')

    markupLogic = slicer.modules.markups.logic()
    if FList.GetNumberOfItems() > 0:  
      markupsList = FList.GetItemAsObject(0)
      markupLogic.SetActiveListID(markupsList)       
    elif AirwayFiducialList.GetNumberOfItems() > 0:
      markupsList = AirwayFiducialList.GetItemAsObject(0)
      markupLogic.SetActiveListID(markupsList) 

    self.enableSelectors()
    self.onSelect()

    return True

  def onSelectFolderButton(self):
    self.disableButtonsAndSelectors()
    self.folderPathSelection.setText(qt.QFileDialog.getExistingDirectory())
    self.enableSelectors()
    self.onSelect()
    
##################################################################################################
################################### CENTERLINE EXTRACTION ######################################## 
##################################################################################################

  def onExtractCenterlineButton(self):
 
    self.disableButtonsAndSelectors()
    
    # Extract Centerline 
    self.extractCenterline(self.labelSelector.currentNode()) 
    
    self.enableSelectors()
    self.onSelect()

    if self.centerlinePointsList != []:
      self.CreateFiducialListButton.enabled = True

    # Update GUI
    self.updateGUI()

  def extractCenterline(self,labelVolume):

    if self.fiducialListSelector.currentNode():  # if a centerline fiducial list was uploaded, all that follows is not necessary!
      self.fiducialNode = self.fiducialListSelector.currentNode()
    elif self.centerlineModelSelector.currentNode():
      self.uploadedCenterlineModel = self.centerlineModelSelector.currentNode()
    else:
      self.centerline = slicer.vtkMRMLScalarVolumeNode()
      slicer.mrmlScene.AddNode( self.centerline )

      centerlineExtraction = slicer.modules.centerlineextractioncli
      parameters = {
          "inputVolume": labelVolume.GetID(),
          "outputVolume": self.centerline.GetID(),	  
          }
      slicer.cli.run( centerlineExtraction,None,parameters,wait_for_completion=True )

      # create 3D model of the centerline
      hierarchyList = slicer.mrmlScene.GetNodesByName('CenterlineModelHierarchy')
      if hierarchyList.GetNumberOfItems() == 0:
        modelHierarchy = slicer.vtkMRMLModelHierarchyNode()
        modelHierarchy.SetName('CenterlineModelHierarchy')
        slicer.mrmlScene.AddNode(modelHierarchy)
      else:
        modelHierarchy = hierarchyList.GetItemAsObject(0)

      parameters = {}
      parameters["InputVolume"] = self.centerline.GetID()
      parameters["ModelSceneFile"] = modelHierarchy.GetID()
      parameters["Name"] = 'CenterlineModel'
      #parameters["FilterType"] = 'Laplacian'
      parameters["Smooth"] = 0
      parameters["Decimate"] = 0.00
    
      modelMaker = slicer.modules.modelmaker
      slicer.cli.run(modelMaker, None, parameters,True)

      # turn off visibility of the created centerline model   
      modelsCollection = slicer.mrmlScene.GetNodesByClass('vtkMRMLModelNode')
      numberOfItems = modelsCollection.GetNumberOfItems()
      self.centerlineModel = modelsCollection.GetItemAsObject(numberOfItems-1)
      displayNode = self.centerlineModel.GetDisplayNode()
      displayNode.SetVisibility(0)

      centerlinePolydata = self.centerlineModel.GetPolyData()

      iterations = 3
      self.Smoothing(centerlinePolydata, iterations)

    if self.fiducialNode:
      disNode = self.fiducialNode.GetDisplayNode()
      disNode.SetVisibility(0)
      for i in xrange(self.fiducialNode.GetNumberOfFiducials()):
        point = [0,0,0]
        self.fiducialNode.GetNthFiducialPosition(i,point)
        self.centerlinePointsList.append(point)
      slicer.mrmlScene.RemoveNode(self.fiducialNode)
    elif self.uploadedCenterlineModel:
      displayNode = self.uploadedCenterlineModel.GetDisplayNode()
      displayNode.SetVisibility(0)
      centerlinePolydata = self.uploadedCenterlineModel.GetPolyData()
      iterations = 3
      self.Smoothing(centerlinePolydata, iterations)
      slicer.mrmlScene.RemoveNode(self.uploadedCenterlineModel)

    self.ProbeTrackButton.enabled = True
    #self.CreateFiducialListButton.enabled = True    

    if self.centerline != None:
      slicer.mrmlScene.RemoveNode(self.centerline)

    return True

  def Smoothing(self, centModel, iterationsNumber):
    
    NumberOfCells = centModel.GetNumberOfCells()
    print centModel.GetNumberOfPoints()

    pointsList = []
    distancePointsAbove = []
    distancePointsBelow = []
    modelPoints = vtk.vtkPoints()

    for iteration in range(0, iterationsNumber):
      if iteration == 0:
        centralPoint = [0,0,0]
        for i in range(NumberOfCells-10,10,-4):
          cell = centModel.GetCell(i)
          points = cell.GetPoints()
          if points.GetNumberOfPoints() % 2 == 0:
            centralPointPosition = points.GetNumberOfPoints()/2
          else:
            centralPointPosition = int(points.GetNumberOfPoints())/2

          points.GetPoint(centralPointPosition,centralPoint)
     
          p = [centralPoint[0],centralPoint[1],centralPoint[2]]
          pointsList.append(p)
          if i == NumberOfCells-10:
            modelPoints.InsertPoint(0, p)
      else:
        point = [0,0,0]
        pointsList = []
        for i in range(0,modelPoints.GetNumberOfPoints()):
          modelPoints.GetPoint(i,point)
          p = [point[0],point[1],point[2]]
          pointsList.append(p) 

      for n in range(1,len(pointsList)-1):
        
        actualPoint = pointsList[n]
        actualPoint = numpy.asarray(actualPoint)

        #
        # closest above point 
        #
        pointsAbove = pointsList[:n]
        followingPointsList = pointsList[n+1:n+200]
        pointsAbove = pointsAbove + followingPointsList
        distancePointsAbove = ((pointsAbove-actualPoint)**2).sum(axis=1)
        ndxAbove = distancePointsAbove.argsort()
        prevPoint =  pointsAbove[ndxAbove[0]]

        applySmooth = 1

        prevFound = 1
        count = 1
        if abs(actualPoint[0]-prevPoint[0]) > 3 and abs(actualPoint[1]-prevPoint[1]) > 2:
          prevFound = 0 
          if abs(actualPoint[2]-prevPoint[2]) > 4:     
            while abs(actualPoint[0]-prevPoint[0]) > 3 and abs(actualPoint[1]-prevPoint[1]) > 2 and abs(actualPoint[2]-prevPoint[2]) > 4 and count < len(ndxAbove) and prevFound == 0:
              prevPoint = pointsAbove[ndxAbove[count]]
              if abs(actualPoint[0]-prevPoint[0]) <= 3 and abs(actualPoint[1]-prevPoint[1]) <= 2 and abs(actualPoint[2]-prevPoint[2]) <= 4:
                 prevFound = 1
              count += 1
          else:
             while abs(actualPoint[0]-prevPoint[0]) > 3 and abs(actualPoint[1]-prevPoint[1]) > 2 and count < len(ndxAbove) and prevFound == 0:
               prevPoint = pointsAbove[ndxAbove[count]]
               if abs(actualPoint[0]-prevPoint[0]) <= 3 and abs(actualPoint[1]-prevPoint[1]) <= 2 and abs(actualPoint[2]-prevPoint[2]) <= 4:
                 prevFound = 1
               count += 1
        elif abs(actualPoint[0]-prevPoint[0]) > 3 or abs(actualPoint[1]-prevPoint[1]) > 2:
          prevFound = 0 
          if abs(actualPoint[0]-prevPoint[0]) > 3:
            while abs(actualPoint[0]-prevPoint[0]) > 3 and count < len(ndxAbove) and prevFound == 0:
              prevPoint = pointsAbove[ndxAbove[count]]
              if abs(actualPoint[0]-prevPoint[0]) <= 3 and abs(actualPoint[1]-prevPoint[1]) <= 2 and abs(actualPoint[2]-prevPoint[2]) <= 4:
                 prevFound = 1
              count += 1
          elif abs(actualPoint[1]-prevPoint[1]) > 2:
            while abs(actualPoint[0]-prevPoint[0]) > 2 and count < len(ndxAbove) and prevFound == 0:
              prevPoint = pointsAbove[ndxAbove[count]]
              if abs(actualPoint[0]-prevPoint[0]) <= 3 and abs(actualPoint[1]-prevPoint[1]) <= 2 and abs(actualPoint[2]-prevPoint[2]) <= 4:
                 prevFound = 1
              count += 1

        # 
        # closest below point
        #
        k = n+1
        pointsBelow = pointsList[k:]
        previousPointsList = pointsList[n-100:n]
        pointsBelow = pointsBelow + previousPointsList
        distancePointsBelow = ((pointsBelow-actualPoint)**2).sum(axis=1)
        ndxBelow = distancePointsBelow.argsort()
        nextPoint = pointsBelow[ndxBelow[0]]      

        count = 1
        nextFound = 1

        if abs(actualPoint[0]-nextPoint[0]) > 3 and abs(actualPoint[1]-nextPoint[1]) > 2:
          nextFound = 0
          if abs(actualPoint[2]-nextPoint[2]) > 4:
	    while abs(actualPoint[0]-nextPoint[0]) > 3 and abs(actualPoint[1]-nextPoint[1]) > 2 and abs(actualPoint[2]-nextPoint[2]) > 4 and count < len(ndxBelow) and nextFound == 0:
              nextPoint = pointsBelow[ndxBelow[count]]
              if abs(actualPoint[0]-nextPoint[0]) <= 3 and abs(actualPoint[1]-nextPoint[1]) <= 2 and abs(actualPoint[2]-nextPoint[2]) <= 4:
                 nextFound = 1
              count += 1
          else:
             while abs(actualPoint[0]-nextPoint[0]) > 3 and abs(actualPoint[1]-nextPoint[1]) > 2 and count < len(ndxBelow) and nextFound == 0:
               nextPoint = pointsBelow[ndxBelow[count]]
               if abs(actualPoint[0]-nextPoint[0]) <= 3 and abs(actualPoint[1]-nextPoint[1]) <= 2 and abs(actualPoint[2]-nextPoint[2]) <= 4:
                 nextFound = 1
               count += 1
        elif abs(actualPoint[0]-nextPoint[0]) > 3 or abs(actualPoint[1]-nextPoint[1]) > 2:
          nextFound = 0
          if abs(actualPoint[0]-nextPoint[0]) > 3:
            while abs(actualPoint[0]-nextPoint[0]) > 3 and count < len(ndxBelow) and nextFound == 0:
              nextPoint = pointsBelow[ndxBelow[count]]
              if abs(actualPoint[0]-nextPoint[0]) <= 3 and abs(actualPoint[1]-nextPoint[1]) <= 2 and abs(actualPoint[2]-nextPoint[2]) <= 4:
                 nextFound = 1
              count += 1
          elif abs(actualPoint[1]-nextPoint[1]) > 2:
            while abs(actualPoint[0]-nextPoint[0]) > 2 and count < len(ndxBelow) and nextFound == 0:
              nextPoint = pointsBelow[ndxBelow[count]]
              if abs(actualPoint[0]-nextPoint[0]) <= 3 and abs(actualPoint[1]-nextPoint[1]) <= 2 and abs(actualPoint[2]-nextPoint[2]) <= 4:
                 nextFound = 1
              count += 1
 
        if nextFound == 0 or prevFound == 0:
          applySmooth = 0

        actualPoint = actualPoint.tolist()

        relaxation = 0.5
        
        if applySmooth == 1:
          actualPoint[0] += relaxation * (0.5 * (prevPoint[0] + nextPoint[0]) - actualPoint[0]);
          actualPoint[1] += relaxation * (0.5 * (prevPoint[1] + nextPoint[1]) - actualPoint[1]);
          actualPoint[2] += relaxation * (0.5 * (prevPoint[2] + nextPoint[2]) - actualPoint[2]);

        if iteration == 0:
          modelPoints.InsertNextPoint(actualPoint)
        else:
          modelPoints.InsertPoint(n, actualPoint)

    print modelPoints.GetNumberOfPoints()
    for i in range(0,modelPoints.GetNumberOfPoints()):
      modelPoints.GetPoint(i,point)
      p = [point[0],point[1],point[2]]
      self.centerlinePointsList.append(p)
    print len(self.centerlinePointsList)

########################################################################################################
######################## Create A Fiducial List With A Fiducial On Each Point ##########################  
########################################################################################################

  def onCreateAndSaveFiducialList(self):

    self.disableButtonsAndSelectors()

    fiducialList = []

    for n in xrange(len(self.centerlinePointsList)):
      point = self.centerlinePointsList[n]
      ID =  'vtkMRMLMarkupsFiducialNode_' + str(n)
      associatedNodeID = 'CenterlineFiducials-' + str(n+1)
      line = [ID,point[0],point[1],point[2],0,0,0,1,1,1,0,associatedNodeID,'','']
      fiducialList.append(line)

    localDirectory = qt.QFileDialog.getExistingDirectory()
    fileName = localDirectory + '/CenterlineFiducials.fcsv'
    a =[]
    with open(fileName, "wb") as f:
      writer = csv.writer(f,)
      version = slicer.app.applicationVersion
      firstRow = '# Markups fiducial file version = ' + str(version[0:3])
      a.append(firstRow)
      writer.writerow(a)
      writer.writerow(['# CoordinateSystem = 0'])
      writer.writerow(['# columns = id']+['x']+['y']+['z']+['ow']+['ox']+['oy']+['oz']+['vis']+['sel']+['lock']+['label']+['desc']+['associatedNodeID'])
      writer.writerows(fiducialList)  

    fiducialList = []
    for n in xrange(len(self.centerlinePointsList)):
      point = self.centerlinePointsList[n]
      line = [point[0],point[1],point[2]]
      fiducialList.append(line)

    fileSecondName = localDirectory + '/CenterlinePositions.txt'
    a = []
    with open(fileSecondName, "wb") as f:
      writer = csv.writer(f,)
      writer.writerows(fiducialList)    

    self.enableSelectors()

    self.onSelect()


#######################################################################################################
##################################### PATH CREATION AND INFO ########################################## 
#######################################################################################################
    
  def onCreateROIFiducialsList(self):

    self.createROIFiducialsButton.setStyleSheet("background-color: rgb(255,99,71)")
    self.createLabelsFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createNewPathPointsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")

    self.alignViewers()
    self.fitSlicesToBackground()
    
    ROIFiducialList = slicer.util.getNode('ROIFiducials')
    markupLogic = slicer.modules.markups.logic()

    if ROIFiducialList:
      markupsList = ROIFiducialList
    else:
      markupsList = slicer.vtkMRMLMarkupsFiducialNode()
      markupsList.SetName('ROIFiducials')
      slicer.mrmlScene.AddNode(markupsList)

    ROIFiducialList = slicer.util.getNode('ROIFiducials')
    displayNode = ROIFiducialList.GetDisplayNode()
    displayNode.SetGlyphScale(5)
    displayNode.SetTextScale(3)

    markupLogic.SetActiveListID(markupsList)
    #self.pointsListSelector.setCurrentNode(markupsList)

    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.SwitchToPersistentPlaceMode()

    self.addFiducialObservers()

  def alignViewers(self):
    crosshairNode = slicer.vtkMRMLCrosshairNode()
    crosshairNode.SetName('viewersAlignmentNode')
    crosshairNode.NavigationOn()
    
    slicer.mrmlScene.AddNode(crosshairNode)
    crosshairNodes = slicer.mrmlScene.GetNodesByName('viewersAlignmentNode')
    crosshairNodes.UnRegister(slicer.mrmlScene)
    crosshairNodes.InitTraversal()
    viewersAlignmentNode = crosshairNodes.GetNextItemAsObject()
    while viewersAlignmentNode:
      viewersAlignmentNode.SetCrosshairMode(1)
      viewersAlignmentNode = crosshairNodes.GetNextItemAsObject()

    self.crosshairNode=slicer.util.getNode('viewersAlignmentNode')
    self.crosshairNode.AddObserver(slicer.vtkMRMLCrosshairNode.CursorPositionModifiedEvent, self.onMouseMoved)
    self.crosshairNode.SetCrosshairMode(5)
    self.crosshairNode.SetCrosshairToMedium()

  def onMouseMoved(self,observer,eventid):
    ras=[0,0,0]
    self.crosshairNode.GetCursorPositionRAS(ras)
    self.crosshairNode.SetCrosshairRAS(ras)

  def fitSlicesToBackground(self):
    lm = slicer.app.layoutManager()
    redWidget = lm.sliceWidget('Red')
    redLogic = redWidget.sliceLogic()
    yellowWidget = lm.sliceWidget('Yellow')
    yellowLogic = yellowWidget.sliceLogic()
    greenWidget = lm.sliceWidget('Green')
    greenLogic = greenWidget.sliceLogic()

    redLogic.FitSliceToBackground(1,1)
    yellowLogic.FitSliceToBackground(1,1)
    greenLogic.FitSliceToBackground(1,1)

  def addFiducialObservers(self):
    '''Add observers to all fiducialLists in scene so we will know when new markups are added'''
    self.removeFiducialObservers()
    fiducialList = slicer.util.getNode('ROIFiducials')
    tag = fiducialList.AddObserver(fiducialList.MarkupAddedEvent, self.requestNodeAddedUpdate)
    self.observeTags.append((fiducialList,tag))

  def removeFiducialObservers(self):
    '''Remove any existing observer'''
    for obj,tag in self.observeTags:
      obj.RemoveObserver(tag)
    self.obsverTags = []

  def requestNodeAddedUpdate(self,caller,event):
    '''Start a SingleShot timer that will check the fiducials in the scene and add them to the list'''
    if not self.pendingUpdate:
      qt.QTimer.singleShot(0,self.wrappedNodeAddedUpdate)
      self.pendingUpdate = True

  def wrappedNodeAddedUpdate(self):
    try:
      self.nodeAddedUpdate()
    except Exception, e:
      import traceback
      traceback.print_exc()
      qt.QMessageBox.warning(slicer.util.mainWindow(),
                             "Node Added", 'Exception!\n\n' + str(e) + "\n\nSee Python Console for Stack Trace")
  def nodeAddedUpdate(self):
    if self.updatingFiducials:
      return
    
    self.updatingFiducials = True
    self.pendingUpdate = False
    self.updatingFiducials = False
    fiducialList = slicer.util.getNode('ROIFiducials')
    if fiducialList:
      self.updateComboBox()

  def updateComboBox(self):
    '''Update the ROIs combobox'''
    fiducialsLogic = slicer.modules.markups.logic()
    activeListID = fiducialsLogic.GetActiveListID()
    activeList = slicer.util.getNode(activeListID)

    if activeList and activeList.GetName() == 'ROIFiducials':
      if activeList.GetNumberOfFiducials() > 0:
        lastElement = activeList.GetNumberOfFiducials() - 1
        self.ROIsPoints.addItem(activeList.GetNthFiducialLabel(lastElement))
        self.ROIsPoints.setCurrentIndex(self.ROIsPoints.count-1)

    self.onPathCreationSelection()

  def showSelectedROI(self):
    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    fidPosition = [0,0,0]
    ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

    lm = slicer.app.layoutManager()
    yellowWidget = lm.sliceWidget('Yellow')
    yellowLogic = yellowWidget.sliceLogic()
    greenWidget = lm.sliceWidget('Green')
    greenLogic = greenWidget.sliceLogic()
    redWidget = lm.sliceWidget('Red')
    redLogic = redWidget.sliceLogic() 
    
    yellowLogic.SetSliceOffset(fidPosition[0])
    greenLogic.SetSliceOffset(fidPosition[1])
    redLogic.SetSliceOffset(fidPosition[2])
    
  def onCreateLabelsFiducialsList(self):

    self.createLabelsFiducialsButton.setStyleSheet("background-color: rgb(255,99,71)")
    self.createROIFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createNewPathPointsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")

    self.fitSlicesToBackground()

    LabelPointFiducialList = slicer.util.getNode('LabelPoints')
    markupLogic = slicer.modules.markups.logic()

    if LabelPointFiducialList:
      markupsList = LabelPointFiducialList
    else:
      markupsList = slicer.vtkMRMLMarkupsFiducialNode()
      markupsList.SetName('LabelPoints')
      slicer.mrmlScene.AddNode(markupsList)

    fidNode =  slicer.util.getNode('LabelPoints')
    fidDisplayNode = fidNode.GetDisplayNode()
    fidDisplayNode.SetGlyphScale(3)
    fidDisplayNode.SetTextScale(0)
    fidDisplayNode.SetSelectedColor(0.0,1.0,1.0)
    
    markupLogic.SetActiveListID(markupsList)
    #self.labelPointsListSelector.setCurrentNode(markupsList)

    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.SwitchToPersistentPlaceMode()

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    fidPosition = [0,0,0]
    ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

    lm = slicer.app.layoutManager()
    yellowWidget = lm.sliceWidget('Yellow')
    yellowLogic = yellowWidget.sliceLogic()
    greenWidget = lm.sliceWidget('Green')
    greenLogic = greenWidget.sliceLogic()
    redWidget = lm.sliceWidget('Red')
    redLogic = redWidget.sliceLogic() 
    
    yellowLogic.SetSliceOffset(fidPosition[0])
    greenLogic.SetSliceOffset(fidPosition[1])
    redLogic.SetSliceOffset(fidPosition[2])
    
    #self.updateGUI()

  def startAddingNewPathPoints(self):

    self.createNewPathPointsButton.setStyleSheet("background-color: rgb(255,99,71)")
    self.createROIFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createLabelsFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")

    self.fitSlicesToBackground()
    
    self.addNewPathPoints = True
    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.SwitchToPersistentPlaceMode()

    fidIndex = self.ROIsPoints.currentIndex
    name = 'AddedPathPointsList-' + str(fidIndex+1)

    AddedPathPointsList = slicer.util.getNode(name)

    if AddedPathPointsList:
      markupsList = AddedPathPointsList
    else:
      markupsList = slicer.vtkMRMLMarkupsFiducialNode()
      markupsList.SetName(name)
      slicer.mrmlScene.AddNode(markupsList)

    AddedPathPointsList = slicer.util.getNode(name)
    displayNode = AddedPathPointsList.GetDisplayNode()
    displayNode.SetGlyphScale(3)
    displayNode.SetTextScale(0)
    displayNode.SetSelectedColor(1.0,1.0,0.0)

    markupLogic = slicer.modules.markups.logic()
    markupLogic.SetActiveListID(markupsList)
    markupLogic.SetActiveListID(markupsList)

    ROIsList = slicer.util.getNode('ROIFiducials')
    fidPosition = [0,0,0]
    ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

    AddedPathPointsList.AddFiducial(fidPosition[0],fidPosition[1],fidPosition[2])

    lm = slicer.app.layoutManager()
    yellowWidget = lm.sliceWidget('Yellow')
    yellowLogic = yellowWidget.sliceLogic()
    greenWidget = lm.sliceWidget('Green')
    greenLogic = greenWidget.sliceLogic()
    redWidget = lm.sliceWidget('Red')
    redLogic = redWidget.sliceLogic() 
    
    yellowLogic.SetSliceOffset(fidPosition[0])
    greenLogic.SetSliceOffset(fidPosition[1])
    redLogic.SetSliceOffset(fidPosition[2])
    
    #self.updateGUI()
  def onDefaultLayoutButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(self.customLayoutId)

    viewNode1 = slicer.util.getNode('vtkMRMLViewNode1')
    viewNode2 = slicer.util.getNode('vtkMRMLViewNode2')
    
    self.firstThreeDView = self.layoutManager.viewWidget(viewNode1).threeDView()
    self.secondThreeDView = self.layoutManager.viewWidget(viewNode2).threeDView()

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if fidIndex >= 0:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()
      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()
      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic()

      yellowLogic.SetSliceOffset(fidPosition[0])
      greenLogic.SetSliceOffset(fidPosition[1])
      redLogic.SetSliceOffset(fidPosition[2])

  def onRedViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(6)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if fidIndex >= 0:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic()

      redLogic.SetSliceOffset(fidPosition[2])
 
  def onYellowViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(7)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if fidIndex >= 0:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()

      yellowLogic.SetSliceOffset(fidPosition[0])

  def onGreenViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(8)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if fidIndex >= 0:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()

      greenLogic.SetSliceOffset(fidPosition[1])

  def onPathCreationButton(self):

    self.createROIFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createLabelsFiducialsButton.setStyleSheet("background-color: rgb(255,255,255)")
    self.createNewPathPointsButton.setStyleSheet("background-color: rgb(255,255,255)")

    if len(self.pathModelNamesList) > 0:
      for n in xrange(len(self.pathModelNamesList)):
        name = self.pathModelNamesList[n]
        model = slicer.util.getNode(name)
        slicer.mrmlScene.RemoveNode(model)
      self.pathModelNamesList = []
    
    labelFiducials = slicer.util.getNode('LabelPoints')

    if labelFiducials:
      self.disableButtonsAndSelectors()

      # Create Centerline Path   
      if self.centerlinePointsList != []:
        self.CreateFiducialListButton.enabled = True
      pos = [0,0,0]
      targetPos = [0,0,0]
      for i in xrange(self.ROIsPoints.count):
        labelFiducials.GetNthFiducialPosition(i,targetPos)
        firstPath = self.pathComputation(self.inputSelector.currentNode(), targetPos)
        
        listName = 'AddedPathPointsList-' + str(i+1)
        AddedPathPointsList = slicer.util.getNode(listName)

        if AddedPathPointsList:
          if AddedPathPointsList.GetNumberOfFiducials() > 0:
            firstPath.GetPoint(0,targetPos)
            AddedPathPointsList.AddFiducial(targetPos[0],targetPos[1],targetPos[2])

            listOfFiducials = []
            distance = []
            p=[0,0,0]
            point=[0,0,0]
            for n in xrange( AddedPathPointsList.GetNumberOfFiducials() ):
              AddedPathPointsList.GetNthFiducialPosition(n,point)
              p = [point[0],point[1],point[2]]
              listOfFiducials.append(p)
            
            listOfFiducials = numpy.asarray(listOfFiducials)
            targetPos = numpy.asarray(targetPos)
            distance = ((listOfFiducials-targetPos)**2).sum(axis=1)
            ndx = distance.argsort()
            orderedList = []
            for t in xrange(len(listOfFiducials)):
              if t==0:
                orderedList.append(listOfFiducials[ndx[t]])
              if t > 0 and (distance[ndx[t]]-distance[ndx[t-1]]) >= 40: # ensure that fiducials are not too close to each other
                orderedList.append(listOfFiducials[ndx[t]])

            if len(orderedList) == 1:
              orderedList.append(listOfFiducials[0])

            computedPath = self.computeAddedPath(orderedList)
            secondPath = self.createAddedPath(computedPath)

          # Merge the two path
          appendFilter = vtk.vtkAppendPolyData()
          appendFilter.AddInputData(firstPath)
          appendFilter.AddInputData(secondPath)
          appendFilter.Update()
        
        ############################ Smooth centerline ###########################
        if AddedPathPointsList:
          createdPath = self.pathSmoothing(appendFilter.GetOutput())
        else:
          createdPath = self.pathSmoothing(firstPath)
        ############################ Make the path thicker #######################
        tubeFilter = vtk.vtkTubeFilter()
        tubeFilter.SetInputData(createdPath)
        tubeFilter.SetRadius(0.12)
        tubeFilter.SetNumberOfSides(50)
        tubeFilter.Update()

        ############################ Create The 3D Model Of The Path And Add It To The Scene ############################################# 

        model = slicer.vtkMRMLModelNode()
        model.SetScene(slicer.mrmlScene)
        model.SetName(slicer.mrmlScene.GenerateUniqueName("PathModel"))
        model.SetAndObservePolyData(tubeFilter.GetOutput())

        # Create display node
        modelDisplay = slicer.vtkMRMLModelDisplayNode()
        modelDisplay.SetColor(0,1,0) # green
        modelDisplay.SetScene(slicer.mrmlScene)
        modelDisplay.LightingOff()
        modelDisplay.SetSliceIntersectionVisibility(1)
        modelDisplay.SetSliceIntersectionThickness(10)
        slicer.mrmlScene.AddNode(modelDisplay)
        model.SetAndObserveDisplayNodeID(modelDisplay.GetID())

        # Add to scene
        if vtk.VTK_MAJOR_VERSION <= 5:
          # shall not be needed.
          modelDisplay.SetInputPolyData(model.GetPolyData())
        slicer.mrmlScene.AddNode(model)

        self.pathModelNamesList.append(model.GetName()) # Save names to delete models before creating the new ones.
      
      self.pathCreated = 1
      
      #slicer.mrmlScene.RemoveNode(labelFiducials)
      labelFiducials.SetDisplayVisibility(0)
      ROINode = slicer.util.getNode('ROIFiducials')

      for i in xrange(ROINode.GetNumberOfFiducials()):
        name = 'AddedPathPointsList-'+str(i+1)
        node = slicer.util.getNode(name)
        #slicer.mrmlScene.RemoveNode(node)
        if node:
          node.SetDisplayVisibility(0)

      self.enableSelectors()
      self.onSelect()

    # Update GUI
    self.updateGUI()
    
    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.Reset()

    self.crosshairNode=slicer.util.getNode('viewersAlignmentNode')
    self.crosshairNode.SetCrosshairMode(0)
    self.crosshairNode.NavigationOff()

    self.fitSlicesToBackground()

  def pathComputation(self, inputModel, targetPosition):
    """
    Run the actual algorithm to create the path between the 2 fiducials
    """
    import vtkSlicerPathExtractionClassesModuleLogic as vmtkLogic
        
    inputPolyData = inputModel.GetPolyData()

    sourceId = vtk.vtkIdList()
    sourceId.SetNumberOfIds(1)

    sourcePosition = self.centerlinePointsList[0]

    source = inputPolyData.FindPoint(sourcePosition)

    sourceId.InsertId(0,source)

    targetId = vtk.vtkIdList()
    targetId.SetNumberOfIds(1)

    for i in xrange(0, 1):  
      target = inputPolyData.FindPoint(targetPosition)
      targetId.InsertId(i,target)

    pathCreation = vmtkLogic.vtkSlicerPathExtractionClassesPolyDataCenterlinesLogic()

    # Multiple paths for different ROIs are created!

    self.createdPath = None
    for t in xrange(targetId.GetNumberOfIds()):

      tempTargetId = vtk.vtkIdList()
      tempTargetId.SetNumberOfIds(1)
      tempTargetId.InsertId(0,targetId.GetId(t))

      pathCreation.SetInputData(inputPolyData)
      pathCreation.SetSourceSeedIds(sourceId)
      pathCreation.SetTargetSeedIds(tempTargetId)
      pathCreation.SetRadiusArrayName('MaximumInscribedSphereRadius')
      pathCreation.SimplifyVoronoiOff();
      pathCreation.CenterlineResamplingOn()
      pathCreation.SetCostFunction('1/R')
      pathCreation.GenerateDelaunayTessellationOn()
      pathCreation.Update()
    
      createdPath = pathCreation.GetOutput()

      #self.pathSmoothing(self.createdPath)

    return pathCreation.GetOutput()
     
  def pathSmoothing(self, pathModel):
      
    import vtkSlicerPathExtractionClassesModuleLogic as vmtkLogic
    
    NumberOfPoints = pathModel.GetNumberOfPoints()
    position = NumberOfPoints-1
    startingPoint = [0,0,0]
    pathModel.GetPoint(position,startingPoint)
    targetPosition = [0,0,0]
    pathModel.GetPoint(1,targetPosition)
           
    squaredDist = vtk.vtkMath.Distance2BetweenPoints(startingPoint,targetPosition)
    
    '''if (squaredDist < 10.000) :
      smoothfactor = 1
      iterations = 100
    else:
      smoothfactor = 1
      iterations = 10'''

    smoothfactor = 1
    iterations = 10
      
    centerlineSmoothing = vmtkLogic.vtkSlicerPathExtractionClassesCenterlineSmoothingLogic()
    centerlineSmoothing.SetInputData(pathModel)
    centerlineSmoothing.SetNumberOfSmoothingIterations(iterations)
    centerlineSmoothing.SetSmoothingFactor(smoothfactor)
    centerlineSmoothing.Update()
    
    return centerlineSmoothing.GetOutput()


  def computeAddedPath(self, fiducialList, dl=0.5):

    self.dl = dl # desired world space step size (in mm)
    self.dt = dl # current guess of parametric stepsize
    self.fids = fiducialList

    # hermite interpolation functions
    self.h00 = lambda t: 2*t**3 - 3*t**2     + 1
    self.h10 = lambda t:   t**3 - 2*t**2 + t
    self.h01 = lambda t:-2*t**3 + 3*t**2
    self.h11 = lambda t:   t**3 -   t**2

    # n is the number of control points in the piecewise curve
    self.n = len(fiducialList)
    n = self.n
    if n == 0:
      return
    # get fiducial positions
    # sets self.p
    self.p = numpy.zeros((n,3))
    for i in xrange(n):
      #coord = [0.0, 0.0, 0.0]
      self.p[i] = self.fids[i]

    # calculate the tangent vectors
    # - fm is forward difference
    # - m is average of in and out vectors
    # - first tangent is out vector, last is in vector
    # - sets self.m
    n = self.n
    fm = numpy.zeros((n,3))
    for i in xrange(0,n-1):
      fm[i] = self.p[i+1] - self.p[i]
    self.m = numpy.zeros((n,3))
    for i in xrange(1,n-1):
      self.m[i] = (fm[i-1] + fm[i]) / 2.
    self.m[0] = fm[0]
    self.m[n-1] = fm[n-2]

    self.AddedPath = [self.p[0]]
    self.calculateAddedPath()
    
    return self.AddedPath

  def calculateAddedPath(self):
    """ Generate a flight path for of steps of length dl """
    #
    # calculate the actual path
    # - take steps of self.dl in world space
    # -- if dl steps into next segment, take a step of size "remainder" in the new segment
    # - put resulting points into self.path
    #
    n = self.n
    segment = 0 # which first point of current segment
    t = 0 # parametric current parametric increment
    remainder = 0 # how much of dl isn't included in current step
    while segment < n-1:
      t, p, remainder = self.addedPathStep(segment, t, self.dl)
      if remainder != 0 or t == 1.:
        segment += 1
        t = 0
        if segment < n-1:
          t, p, remainder = self.addedPathStep(segment, t, remainder)
      self.AddedPath.append(p)

  def AddedPoint(self,segment,t):
    return (self.h00(t)*self.p[segment] +
              self.h10(t)*self.m[segment] +
              self.h01(t)*self.p[segment+1] +
              self.h11(t)*self.m[segment+1])

  def addedPathStep(self,segment,t,dl):
    """ Take a step of dl and return the path point and new t
      return:
      t = new parametric coordinate after step
      p = point after step
      remainder = if step results in parametic coordinate > 1.0, then
        this is the amount of world space not covered by step
    """
    p0 = self.AddedPath[self.AddedPath.__len__() - 1] # last element in path
    remainder = 0
    ratio = 100
    count = 0
    while abs(1. - ratio) > 0.05:
      t1 = t + self.dt
      pguess = self.AddedPoint(segment,t1)
      dist = numpy.linalg.norm(pguess - p0)
      ratio = self.dl / dist
      self.dt *= ratio
      if self.dt < 0.00000001:
        return
      count += 1
      if count > 500:
        return (t1, pguess, 0)
    if t1 > 1.:
      t1 = 1.
      p1 = self.AddedPoint(segment, t1)
      remainder = numpy.linalg.norm(p1 - pguess)
      pguess = p1
    return (t1, pguess, remainder)

  def createAddedPath(self,path):

    scene = slicer.mrmlScene

    points = vtk.vtkPoints()
    polyData = vtk.vtkPolyData()
    polyData.SetPoints(points)

    lines = vtk.vtkCellArray()
    polyData.SetLines(lines)
    linesIDArray = lines.GetData()
    linesIDArray.Reset()
    linesIDArray.InsertNextTuple1(0)

    polygons = vtk.vtkCellArray()
    polyData.SetPolys( polygons )
    idArray = polygons.GetData()
    idArray.Reset()
    idArray.InsertNextTuple1(0)

    for point in path:
      pointIndex = points.InsertNextPoint(*point)
      linesIDArray.InsertNextTuple1(pointIndex)
      linesIDArray.SetTuple1( 0, linesIDArray.GetNumberOfTuples() - 1 )
      lines.SetNumberOfCells(1)

    return polyData
   
  def onPathSelect(self):
    #self.updateGUI()
    # Hide all paths and fiducials...
    if len(self.pathModelNamesList) > 0:
      for n in xrange(len(self.pathModelNamesList)):
        pathName = self.pathModelNamesList[n]
        pathModel = slicer.util.getNode(pathName)
        displayNode = pathModel.GetDisplayNode()
        displayNode.SetVisibility(0)

    fidNode =  slicer.util.getNode('ROIFiducials')
    fidDisplayNode = fidNode.GetDisplayNode()
    fidDisplayNode.SetGlyphScale(8)
    fidDisplayNode.SetTextScale(8)
    
    for i in xrange(fidNode.GetNumberOfFiducials()):        
      fidNode.SetNthFiducialVisibility(i,0)
      fidNode.SetNthFiducialLabel(i,str(i+1))
     
    # ...and show only the selected one
    pathModel = self.pathModelSelector.currentNode()
    displayNode = pathModel.GetDisplayNode()
    displayNode.SetVisibility(1)

    # Merge Centerline Points with Path Points 
    if self.centerlinePointsList != []:
      pathPolydata = pathModel.GetPolyData()
      pathPoints = pathPolydata.GetPoints()
      for j in xrange(pathPoints.GetNumberOfPoints()):
        point = pathPoints.GetPoint(j)
        p = [point[0],point[1],point[2]]
        self.centerlinePointsList.append(point)

      # Avoid repetition of the same point twice
      self.centerlinePointsList = numpy.array([list(x) for x in set(tuple(x) for x in self.centerlinePointsList)])
      self.centerlinePointsList = self.centerlinePointsList.tolist()

    # Display fiducial corresponding to the selected path
    name = pathModel.GetName()
    idx = self.pathModelNamesList.index(name)
    #fidNode =  self.pointsListSelector.currentNode()
    fidNode.SetNthFiducialVisibility(idx,1)

    self.pathInfo(pathModel, fidDisplayNode)

  def pathInfo(self, model, dispNode):
    polyData = model.GetPolyData()
    numberOfPoints = polyData.GetNumberOfPoints()
    
    firstPoint = [0,0,0]
    polyData.GetPoint(numberOfPoints-1, firstPoint)
    lastPoint = [0,0,0]
    polyData.GetPoint(1, lastPoint)

    squaredLength = vtk.vtkMath.Distance2BetweenPoints(firstPoint, lastPoint)
    length = math.sqrt(squaredLength)
    length = int(length)
    if length == 0:
      dispNode.SetSelectedColor(0.22,1.0,1.0)
    else:
      dispNode.SetSelectedColor(1.0,0.5,0.5)
    length = str(length) + ' mm'        
       
    self.pathLength.setText(length)

  def delayDisplay(self,message,msec=1000):
    #
    # logic version of delay display
    #
    print(message)
    self.info = qt.QDialog()
    self.infoLayout = qt.QVBoxLayout()
    self.info.setLayout(self.infoLayout)
    self.label = qt.QLabel(message,self.info)
    self.infoLayout.addWidget(self.label)
    qt.QTimer.singleShot(msec, self.info.close)
    self.info.exec_()
  
###########################################################################################
#################################### SENSOR TRACKING ######################################
###########################################################################################

  def onProbeTrackButtonToggled(self, checked):     
    if checked:
      self.updateGUI()
      self.ImageRegistrationButton.show()

      # if probeModel was not loaded a warning will appear and tracking will be stopped 
      probeNode = slicer.util.getNode('ProbeModel')
      pathModel = self.pathModelSelector.currentNode()      
      if probeNode == None:
        messageBox = qt.QMessageBox()
        messageBox.warning(None,'Warning!', 'Please add ProbeModel before starting the sensor tracking!')
        self.ProbeTrackButton.checked = False
      elif pathModel == None:
        messageBox = qt.QMessageBox()
        messageBox.warning(None,'Warning!', 'Please select a path before starting the sensor tracking!')
        self.ProbeTrackButton.checked = False
      else:
        # Hide registration markers, if any
        regMarkers = slicer.util.getNode('RegistrationMarker')
        if regMarkers:
          regMarkers.SetDisplayVisibility(0)

        self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,156,126)")

        self.disableButtonsAndSelectors()
        self.ProbeTrackButton.enabled = True
        self.FlipImageButton.enabled = True

        self.ProbeTrackButton.text = "Stop Tracking"

        if self.cNode == None:
          cNodes = slicer.mrmlScene.GetNodesByName('ProbeConnector')
          if cNodes.GetNumberOfItems() == 0:
            self.cNode = slicer.vtkMRMLIGTLConnectorNode()
            slicer.mrmlScene.AddNode(self.cNode)
            self.cNode.SetName('ProbeConnector')
          else:
            self.cNode = cNodes.GetItemAsObject(0)

        self.cNode.SetType(1)
        self.cNode.SetTypeServer(18944)
        self.cNode.Start()

        ################## Transform matrix to compensate for possible flipping of the 3D image #####################

        if self.flipCompensationTransform == None:
          flipCompensationTransformNodes = slicer.mrmlScene.GetNodesByName('flipCompensationTransform')
          if flipCompensationTransformNodes.GetNumberOfItems() == 0:
            self.flipCompensationTransform = slicer.vtkMRMLLinearTransformNode()
            self.flipCompensationTransform.SetName('flipCompensationTransform')
            slicer.mrmlScene.AddNode(self.flipCompensationTransform)
          else:
	    self.flipCompensationTransform = flipCompensationTransformNodes.GetItemAsObject(0)

        ################## This turns the probe of 90 degrees when the tracking is started the first time #####################

        if self.probeCalibrationTransform == None:
          calibrationTransformNodes = slicer.mrmlScene.GetNodesByName('probeCalibrationTransform')
          if calibrationTransformNodes.GetNumberOfItems() == 0:
            self.probeCalibrationTransform = slicer.vtkMRMLLinearTransformNode()
            self.probeCalibrationTransform.SetName('probeCalibrationTransform')
            slicer.mrmlScene.AddNode(self.probeCalibrationTransform)
          else:
            self.probeCalibrationTransform = calibrationTransformNodes.GetItemAsObject(0)

        calibrationMatrix = vtk.vtkMatrix4x4()
        self.probeCalibrationTransform.GetMatrixTransformToParent(calibrationMatrix)
        calibrationMatrix.SetElement(0,0,0)
        calibrationMatrix.SetElement(0,2,1)
        calibrationMatrix.SetElement(2,0,-1)
        calibrationMatrix.SetElement(2,2,0)
        self.probeCalibrationTransform.SetMatrixTransformToParent(calibrationMatrix)
 
        self.probeCalibrationTransform.SetAndObserveTransformNodeID(self.flipCompensationTransform.GetID())

        # Transform to compensate probe position with with centerline points

        if self.centerlineCompensationTransform == None:
          centerlineCompensationTransformNodes = slicer.mrmlScene.GetNodesByName('centerlineCompensationTransform')
          if centerlineCompensationTransformNodes.GetNumberOfItems() == 0:
            self.centerlineCompensationTransform = slicer.vtkMRMLLinearTransformNode()
            self.centerlineCompensationTransform.SetName('centerlineCompensationTransform')
            slicer.mrmlScene.AddNode(self.centerlineCompensationTransform)
          else:
            self.centerlineCompensationTransform = centerlineCompensationTransformNodes.GetItemAsObject(0)

        if probeNode:
          probeDisplayNode = probeNode.GetDisplayNode()
          probeDisplayNode.SetColor(0.9215686274509803, 0.03137254901960784, 1.0)
          probeDisplayNode.SetSliceIntersectionVisibility(1)
          probeDisplayNode.SetSliceIntersectionThickness(7)

          ########## A fiducial is created to indicate the position of the probe in saggital, coronal and axial views ##########

          #probePositionIndicator = slicer.vtkMRMLMarkupsFiducialNode()
          #probePositionIndicator.SetName('ProbePositionIndicator')
          #slicer.mrmlScene.AddNode(probePositionIndicator)
          # The fiducial is placed on the tip of the probe
          #probePositionIndicator.AddFiducial(-1.0,-0.0,0.0)
          #probeIndicatorDisplayNode = probePositionIndicator.GetDisplayNode()
          #probeIndicatorDisplayNode.SetGlyphScale(6.0)
          #probeIndicatorDisplayNode.SetGlyphType(10)
          #probeIndicatorDisplayNode.SetTextScale(0.0)

          #######################################################################################################################

          if probeNode.GetTransformNodeID() == None:
            probeNode.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
            #probePositionIndicator.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())

        ################## Camera 1 and 3 (if any) are initialized and connected to the transform #####################
         
        self.initializeCamera()
        self.cameraForNavigation.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
        if self.thirdCamera:
          self.thirdCamera.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
      
        ####################### Set clipping range for first and third (if any) cameras ####################
        camera = self.cameraForNavigation.GetCamera()
        camera.SetClippingRange(0.7081381565016212, 708.1381565016211) # set a random clipping range to force the camera to reset it 

        if self.thirdCamera:
          thirdCamera = self.thirdCamera.GetCamera()
          thirdCamera.SetClippingRange(0.7081381565016212, 708.1381565016211)

        ########## Camera 2 is will follow the probe from outside the model ##########
        cameraNodes = slicer.mrmlScene.GetNodesByName('Default Scene Camera')
        self.secondCamera = cameraNodes.GetItemAsObject(1)
        self.secondCamera.SetFocalPoint(-1.0,0.0,0.0)
        secCamera = self.secondCamera.GetCamera()
        secCamera.SetClippingRange(0.5741049687312555, 574.1049687312554)

        ####### Red, yellow, and green positions are modified to follow the probe on the CT ###### 
 
        lm = slicer.app.layoutManager()
        yellowWidget = lm.sliceWidget('Yellow')
        self.yellowLogic = yellowWidget.sliceLogic()
        redWidget = lm.sliceWidget('Red')
        self.redLogic = redWidget.sliceLogic() 
        greenWidget = lm.sliceWidget('Green')
        self.greenLogic = greenWidget.sliceLogic()
 
        self.sensorTimer.start()
       
        self.layoutManager = slicer.app.layoutManager()
        #self.firstThreeDView = self.layoutManager.threeDWidget( 0 ).threeDView()
        self.firstViewCornerAnnotation = self.firstThreeDView.cornerAnnotation()
        self.secondViewCornerAnnotation = self.secondThreeDView.cornerAnnotation()
        self.thirdViewCornerAnnotation = None
        if self.thirdThreeDView: 
          self.thirdViewCornerAnnotation = self.thirdThreeDView.cornerAnnotation()        
       
    else:  # When button is released...      
      self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.FlipImageButton.enabled = False
      self.ImageRegistrationButton.hide()

      self.sensorTimer.stop()
      
      if self.cNode:
        self.cNode.Stop()

        self.cameraForNavigation.GetFocalPoint(self.lastFPBeforeStoppingTracking)
        self.cameraForNavigation.GetPosition(self.lastPosBeforeStoppingTracking)
        self.cameraForNavigation.GetViewUp(self.lastViewUp)
        self.cameraForNavigation.SetFocalPoint(self.lastFPBeforeStoppingTracking[0],self.lastFPBeforeStoppingTracking[1],self.lastFPBeforeStoppingTracking[2])
        self.cameraForNavigation.SetPosition(self.lastPosBeforeStoppingTracking[0],self.lastPosBeforeStoppingTracking[1],self.lastPosBeforeStoppingTracking[2])
        camera = self.cameraForNavigation.GetCamera()  
        self.cameraForNavigation.SetViewUp(self.lastViewUp)
        camera.SetClippingRange(0.7081381565016212, 708.1381565016211)

        cameraNodes = slicer.mrmlScene.GetNodesByName('Default Scene Camera')
        self.secondCamera = cameraNodes.GetItemAsObject(1)
        secCamera = self.secondCamera.GetCamera()
        secCamera.SetClippingRange(0.5741049687312555, 574.1049687312554)

      self.enableSelectors()
      self.onSelect()

      if self.centerlinePointsList != []:
        self.CreateFiducialListButton.enabled = True

      self.ProbeTrackButton.text = "Track Sensor"      

  def ReadPosition(self):
      if self.cNode.GetState() == 2:
        transformNodesCollection = slicer.mrmlScene.GetNodesByName('ProbeToTracker')
        if self.probeToTrackerTransformNode == None:
          self.probeToTrackerTransformNode = transformNodesCollection.GetItemAsObject(0)        

	###################### Centerline Compensation #########################

        if self.probeToTrackerTransformNode:
          transformMatrix = vtk.vtkMatrix4x4()
          self.probeToTrackerTransformNode.GetMatrixTransformToParent(transformMatrix)

          #if self.probeCalibrationTransform.GetTransformNodeID() == None:
            #self.probeCalibrationTransform.SetAndObserveTransformNodeID(self.centerlineCompensationTransform.GetID())
          if self.flipCompensationTransform.GetTransformNodeID() == None:
            self.flipCompensationTransform.SetAndObserveTransformNodeID(self.centerlineCompensationTransform.GetID())

          self.CheckCurrentPosition(transformMatrix)
  
  def initializeCamera(self):
    cameraNodes = slicer.mrmlScene.GetNodesByName('Default Scene Camera')
    self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
    if cameraNodes.GetNumberOfItems() > 0:
      if self.cameraForNavigation.GetTransformNodeID() == None:
        self.cameraForNavigation.SetPosition(-1.0,0.0,0.0)
        self.cameraForNavigation.SetFocalPoint(-5.0,0.0,.0)
        viewUp = [0.0,0.0,-1.0]
        self.cameraForNavigation.SetViewUp(viewUp)
      else:
        if self.lastFPBeforeStoppingTracking != [0,0,0] and self.lastPosBeforeStoppingTracking != [0,0,0] and self.lastViewUp != [0,0,0]:
          self.cameraForNavigation.SetFocalPoint(self.lastFPBeforeStoppingTracking[0],self.lastFPBeforeStoppingTracking[1],self.lastFPBeforeStoppingTracking[2])
          self.cameraForNavigation.SetPosition(self.lastPosBeforeStoppingTracking[0],self.lastPosBeforeStoppingTracking[1],self.lastPosBeforeStoppingTracking[2])
          camera = self.cameraForNavigation.GetCamera()  
          self.cameraForNavigation.SetViewUp(self.lastViewUp)
          camera.SetClippingRange(0.7081381565016212, 708.1381565016211)

          self.lastFPBeforeStoppingTracking  = [0,0,0]
          self.lastPosBeforeStoppingTracking = [0,0,0]
          self.lastViewUp = [0,0,0]

      self.cameraForNavigation.SetViewAngle(50)

    if self.thirdThreeDView:
      self.thirdCamera = cameraNodes.GetItemAsObject(2)
      if cameraNodes.GetNumberOfItems() > 0:
        if self.thirdCamera.GetTransformNodeID() == None:
          f = [0.0,0.0,0.0]
          self.cameraForNavigation.GetPosition(f)
          f[0] += 20
	  f[1] += 5        
          self.thirdCamera.SetPosition(f)
          self.cameraForNavigation.GetFocalPoint(f)
          self.thirdCamera.SetFocalPoint(f)
          viewUp = [0.0,0.0,-1.0]
          self.thirdCamera.SetViewUp(viewUp)
        else:
          if self.lastFPBeforeStoppingTracking != [0,0,0] and self.lastPosBeforeStoppingTracking != [0,0,0] and self.lastViewUp != [0,0,0]:
            self.thirdCamera.SetFocalPoint(self.lastFPBeforeStoppingTracking[0],self.lastFPBeforeStoppingTracking[1],self.lastFPBeforeStoppingTracking[2])
            self.thirdCamera.SetPosition(self.lastPosBeforeStoppingTracking[0],self.lastPosBeforeStoppingTracking[1],self.lastPosBeforeStoppingTracking[2])
            thirdCamera = self.thirdCamera.GetCamera()  
            self.thirdCamera.SetViewUp(self.lastViewUp)
            thirdCamera.SetClippingRange(0.7081381565016212, 708.1381565016211)

            self.lastFPBeforeStoppingTracking  = [0,0,0]
            self.lastPosBeforeStoppingTracking = [0,0,0]
            self.lastViewUp = [0,0,0]

        self.thirdCamera.SetViewAngle(55)
        self.thirdCameraInitialized = 1

  def onChangeLayoutButtonToggled(self, checked):
    if checked:
      self.newLayoutImageButton.text = "Return To Default Layout"  
      self.fitSlicesToBackground()
      self.layoutManager.setLayout(self.three3DViewsLayoutId)
      if self.firstThreeDView == None:
        viewNode1 = slicer.util.getNode('vtkMRMLViewNode1')
        self.firstThreeDView = self.layoutManager.viewWidget(viewNode1).threeDView()
      if self.secondThreeDView == None:
        viewNode2 = slicer.util.getNode('vtkMRMLViewNode2')
        self.secondThreeDView = self.layoutManager.viewWidget( viewNode2 ).threeDView()

      viewNode3 = slicer.util.getNode('vtkMRMLViewNode3')
      self.thirdThreeDView = self.layoutManager.viewWidget( viewNode3 ).threeDView()

      cameraNodes = slicer.mrmlScene.GetNodesByName('Default Scene Camera')
      self.thirdCamera = cameraNodes.GetItemAsObject(2)

      if self.probeCalibrationTransform:
        self.thirdCamera.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())

      thirdCamera = self.thirdCamera.GetCamera()
      thirdCamera.SetClippingRange(0.7081381565016212, 708.1381565016211)

    else:
      self.newLayoutImageButton.text = "Add Third 3D View"  
      self.onDefaultLayoutButton()


  def onFlipImageButton(self):
    if self.flipCompensationTransform:
      flipMatrix = vtk.vtkMatrix4x4()
      self.flipCompensationTransform.GetMatrixTransformToParent(flipMatrix)
      firstElement = flipMatrix.GetElement(0,0)
      fifthElement = flipMatrix.GetElement(1,1)

      changeSign = -1
      flipMatrix.SetElement(0,0,firstElement*changeSign)
      flipMatrix.SetElement(1,1,fifthElement*changeSign)

      self.flipCompensationTransform.SetMatrixTransformToParent(flipMatrix)

  def CheckCurrentPosition(self, tMatrix):

    #################################
    ####### Check translation #######
    #################################
    
    distance = []

    originalCoord = [0.0,0.0,0.0]
    originalCoord[0] = tMatrix.GetElement(0,3)
    originalCoord[1] = tMatrix.GetElement(1,3)
    originalCoord[2] = tMatrix.GetElement(2,3)

    originalCoord = numpy.asarray(originalCoord)
    self.centerlinePointsList = numpy.asarray(self.centerlinePointsList)

    distance = ((self.centerlinePointsList-originalCoord)**2).sum(axis=1)
    ndx = distance.argsort()
    closestPoint = self.centerlinePointsList[ndx[0]]

    tMatrix.SetElement(0,3,closestPoint[0])
    tMatrix.SetElement(1,3,closestPoint[1])
    tMatrix.SetElement(2,3,closestPoint[2])

    self.centerlinePointsList = self.centerlinePointsList.tolist()

    ##################################################
    ############ Keep rotation constant ##############
    ##################################################
    ISRotation = []
    
    firstRow  = [0.0,0.0]
    secondRow = [0.0,0.0]
    thirdRow  = [0.0,0.0]

    firstRow[0] = tMatrix.GetElement(0,0)
    firstRow[1] = tMatrix.GetElement(0,1)

    ISRotation.append(firstRow)

    secondRow[0] = tMatrix.GetElement(1,0)
    secondRow[1] = tMatrix.GetElement(1,1)

    ISRotation.append(secondRow)

    thirdRow[0] = tMatrix.GetElement(2,0)
    thirdRow[1] = tMatrix.GetElement(2,1)

    ISRotation.append(thirdRow)

    ISRotation = numpy.asarray(ISRotation)
    newMatrixSigns = numpy.sign(ISRotation)

    if len(self.previousMatrixSigns) == 0:
      self.previousMatrixSigns = newMatrixSigns

    tMatrix.SetElement(0,0,abs(firstRow[0])  * self.previousMatrixSigns[0,0])
    tMatrix.SetElement(0,1,abs(firstRow[1])  * self.previousMatrixSigns[0,1])
    tMatrix.SetElement(1,0,abs(secondRow[0]) * self.previousMatrixSigns[1,0])
    tMatrix.SetElement(1,1,abs(secondRow[1]) * self.previousMatrixSigns[1,1])
    tMatrix.SetElement(2,0,abs(thirdRow[0])  * self.previousMatrixSigns[2,0])
    tMatrix.SetElement(2,1,abs(thirdRow[1])  * self.previousMatrixSigns[2,1])

    ####################################################################################################################
    # Continuosly Update ViewUp Of The Camera To Always Have It On One Direction Orthogonal To The Locator's Long Axis #
    ####################################################################################################################

    x = closestPoint[0]
    y = closestPoint[1]
    z = closestPoint[2]          
    #c = (x+y)/z
    #viewUp = [1,1,c]
    #self.cameraForNavigation.SetViewUp(viewUp)

    self.yellowLogic.SetSliceOffset(x)
    self.greenLogic.SetSliceOffset(y)
    self.redLogic.SetSliceOffset(z)

    self.centerlineCompensationTransform.SetMatrixTransformToParent(tMatrix)

    # force the camera position to be a bit higher to better watch the path
    self.cameraForNavigation.SetPosition(x,y,z-1)

    if self.thirdCamera:
      if self.thirdCameraInitialized == 0:
        self.thirdCamera.SetPosition(x-20,y+15,z+70)
      fp = [0.0,0.0,0.0]
      self.cameraForNavigation.GetFocalPoint(fp)
      self.thirdCamera.SetFocalPoint(fp)
      viewUp = [0.0,0.0,0.0]
      self.cameraForNavigation.GetViewUp(viewUp)
      self.thirdCamera.SetViewUp(viewUp)

    if len(self.pathModelNamesList) > 0:
      pathModel = self.pathModelSelector.currentNode()
      pathPolyData = pathModel.GetPolyData()
      self.distanceToTargetComputation(pathPolyData, closestPoint)

    pos = [0,0,0]
    self.secondCamera.SetFocalPoint(x,y,z)
    
    if self.length > 90:
      self.secondCamera.SetPosition(x,y+600,z)
    elif 90 <= self.length < 70:
       self.secondCamera.SetPosition(x,y+400,z)      
    else:
      self.secondCamera.SetPosition(x,y+150,z)

    secCamera = self.secondCamera.GetCamera()
    secCamera.SetClippingRange(0.5741049687312555, 574.1049687312554)

    ####################################################################################################################
    ####################### If requested start image registration (at bifurcation points) ##############################
    ####################################################################################################################
    elapsedTime = time.time() - self.time
    if self.bifurcationPointsList != [] and elapsedTime >= 3:
      self.bifurcationPointsList = numpy.asarray(self.bifurcationPointsList)
      closestPoint = numpy.asarray(closestPoint)

      euclDist = ((self.bifurcationPointsList-closestPoint)**2).sum(axis=1)
      minDist = min(euclDist)

      print minDist
      if 20 <= minDist <= 30:
        self.registerImage()

      self.bifurcationPointsList = self.bifurcationPointsList.tolist()
      closestPoint = closestPoint.tolist()
      self.time = time.time()

  def distanceToTargetComputation(self, polyData, secondPoint):

    numberOfPoints = polyData.GetNumberOfPoints()
    
    firstPoint = [0,0,0]
    polyData.GetPoint(numberOfPoints-1, firstPoint)

    squaredDistance = vtk.vtkMath.Distance2BetweenPoints(firstPoint, secondPoint)
    self.length = math.sqrt(squaredDistance)
    self.length = int(self.length)
    
    # Change color of the fiducial when close to the ROI
    ROIFiducialList = slicer.util.getNode('ROIFiducials')
    displayNode = ROIFiducialList.GetDisplayNode()
    if self.length <= 4:
      displayNode.SetSelectedColor(0.4, 1.0, 1.0)
    else:
      displayNode.SetSelectedColor(1.0,0.0,0.0)      
    
    string_length = str(self.length) + ' mm'        
       
    self.distanceToTarget.setText(string_length)
    
    distToTarget = 'Distance To Target: ' + string_length

    self.firstViewCornerAnnotation.SetText(1,distToTarget)
    self.secondViewCornerAnnotation.SetText(1,distToTarget)
    if self.thirdViewCornerAnnotation:
      self.thirdViewCornerAnnotation.SetText(1,distToTarget)
    
    color = qt.QColor('yellow')
    firstTxtProperty = self.firstViewCornerAnnotation.GetTextProperty()
    firstTxtProperty.SetColor(color.redF(), color.greenF(), color.blueF())
    firstTxtProperty.SetBold(1)
    #firstTxtProperty.SetFontFamilyAsString('Courier')
    
    secondTxtProperty = self.secondViewCornerAnnotation.GetTextProperty()
    secondTxtProperty.SetColor(color.redF(), color.greenF(), color.blueF())
    secondTxtProperty.SetBold(1)
    #secondTxtProperty.SetFontFamilyAsString('Courier')

    self.secondThreeDView.forceRender()

    if self.thirdViewCornerAnnotation:
      thirdTxtProperty = self.thirdViewCornerAnnotation.GetTextProperty()
      thirdTxtProperty.SetColor(color.redF(), color.greenF(), color.blueF())
      thirdTxtProperty.SetBold(1)
      #thirdTxtProperty.SetFontFamilyAsString('Courier')
      self.thirdThreeDView.forceRender()

  def startVideoStreaming(self, checked):
    if checked:
      # Show button to start image registration
      self.ImageRegistrationButton.enabled = True

      # start streaming video
      self.VideoRegistrationButton.setText("Stop Video Streaming")
      self.VideoRegistrationButton.setStyleSheet("background-color: rgb(215,255,255)")
      if self.videoStreamingNode == None:
        streamingNodes = slicer.mrmlScene.GetNodesByName('streamingConnector')
        if streamingNodes.GetNumberOfItems() == 0:
          self.videoStreamingNode = slicer.vtkMRMLIGTLConnectorNode()
          slicer.mrmlScene.AddNode(self.videoStreamingNode)
          self.videoStreamingNode.SetName('streamingConnector')
        else:
          self.videoStreamingNode = streamingNodes.GetItemAsObject(0)

        self.videoStreamingNode.SetTypeClient('localhost',18945)
        self.videoStreamingNode.Start()

        self.checkStreamingTimer.start()
    else:

      # Stop image registration and hide button 
      self.ImageRegistrationButton.checked = False
      self.ImageRegistrationButton.enabled = False

      if self.videoStreamingNode != None:
        self.VideoRegistrationButton.setText("Start Video Streaming")
        self.VideoRegistrationButton.setStyleSheet("background-color: rgb(255,255,255)")
        self.videoStreamingNode.Stop()

  def showVideoStreaming(self):
    if self.videoStreamingNode != None: 
      if self.videoStreamingNode.GetState() == 2:
        videoNode = slicer.util.getNode('Image_Reference')
        if videoNode:
          realViewWidget = self.layoutManager.sliceWidget('RealView')
          RVLogic = realViewWidget.sliceLogic()
          RV_cn = RVLogic.GetSliceCompositeNode()
          RV_cn.SetBackgroundVolumeID(videoNode.GetID())
          RVLogic.FitSliceToVolume(videoNode,1,1)
          self.checkStreamingTimer.stop()

  ###########################################################################################
  ################################## Image Registration #####################################
  ###########################################################################################
  def onStartImageRegistrationButtonPressed(self, checked):
    if checked: 
      fileName = qt.QFileDialog.getOpenFileName()
      print fileName
      fileID = open(fileName, 'r')
      for line in fileID:
        line = eval('['+line+']')
        self.bifurcationPointsList.append(line)

      self.time = time.time()
      self.ImageRegistrationButton.text = "Stop Image Registration"
    else:
      self.bifurcationPointsList = []
      self.ImageRegistrationButton.text = "Start Image Registration"

  def registerImage(self):
    # Read the real image
    videoNode = slicer.util.getNode('Image_Reference')

    # Crop image to remove the info part on the left side
    VOIExtract = vtk.vtkExtractVOI()
    VOIExtract.SetInputConnection(videoNode.GetImageDataConnection())
    VOIExtract.SetVOI(180,571,58,430,0,0)
    VOIExtract.Update()
    
    '''w = vtk.vtkPNGWriter()
    w.SetInputConnection(VOIExtract.GetOutputPort())

    w.SetFileName('C:/Users/Lab/Desktop/text.png')
    w.Write()'''
    
    # Flip image about x
    realImageFlipX = vtk.vtkImageFlip()
    realImageFlipX.SetFilteredAxis(0)
    realImageFlipX.SetInputConnection(VOIExtract.GetOutputPort())
    #realImageFlipX.SetInputConnection(videoNode.GetImageDataConnection())

    realImageFlipX.Update()
    
    # Convert image to gray-scale 
    realExtract = vtk.vtkImageExtractComponents()
    realExtract.SetComponents(0,1,2)
    realLuminance = vtk.vtkImageLuminance()
    
    if vtk.VTK_MAJOR_VERSION <= 5:
      realExtract.SetInput(realImageFlipX.GetOutput())
      realLuminance.SetInput(realExtract.GetOutput())
      realLuminance.GetOutput().Update() 
    else:
      realExtract.SetInputConnection(realImageFlipX.GetOutputPort())
      realLuminance.SetInputConnection(realExtract.GetOutputPort())
      realLuminance.Update()

    realScalarVolume = slicer.vtkMRMLScalarVolumeNode()
    realScalarVolume.SetName('fixedScalarImage')

    if vtk.VTK_MAJOR_VERSION <= 5:
      realScalarVolume.SetImageData(realLuminance.GetOutput())
    else:
      realScalarVolume.SetImageDataConnection(realLuminance.GetOutputPort())

    slicer.mrmlScene.AddNode(realScalarVolume)

    # Grab 3D view
    pathModel = self.pathModelSelector.currentNode()
    displayNode = pathModel.GetDisplayNode()

    ROIfids = slicer.util.getNode('ROIFiducials')
    fidsDisplayNode = ROIfids.GetDisplayNode()
    fidsDisplayNode.SetVisibility(0)
    
    rw = self.firstThreeDView.renderWindow()
    wti = vtk.vtkWindowToImageFilter()
    wti.SetInput(rw)
    displayNode.SetVisibility(0)
    slicer.app.processEvents()
    wti.Update()
    displayNode.SetVisibility(1)
    fidsDisplayNode.SetVisibility(1)
    slicer.app.processEvents()
    
    # Convert image to gray-scale
    movingExtract = vtk.vtkImageExtractComponents()
    movingExtract.SetComponents(0,1,2)
    movingLuminance = vtk.vtkImageLuminance()
    if vtk.VTK_MAJOR_VERSION <= 5:
      movingExtract.SetInput(wti.GetOutput())
      movingLuminance.SetInput(movingExtract.GetOutput())
      movingLuminance.GetOutput().Update() 
    else:
      movingExtract.SetInputConnection(wti.GetOutputPort())
      movingLuminance.SetInputConnection(movingExtract.GetOutputPort())
      movingLuminance.Update()

    # Flip image about x
    movingImageFlip = vtk.vtkImageFlip()
    movingImageFlip.SetFilteredAxis(0)
    movingImageFlip.SetInputData(movingLuminance.GetOutput())
    movingImageFlip.Update()

    movingScalarVolume = slicer.vtkMRMLScalarVolumeNode()
    movingScalarVolume.SetName('movingScalarImage')
   
    if vtk.VTK_MAJOR_VERSION <= 5:
      movingScalarVolume.SetImageData(movingImageFlip.GetOutput())
    else:
      movingScalarVolume.SetAndObserveImageData(movingImageFlip.GetOutput())

    slicer.mrmlScene.AddNode(movingScalarVolume)

    anglesNumber = 36
    imageRegistration = slicer.modules.imageregistrationcli
    parameters = {
          "fixedImage": realScalarVolume,
          "movingImage": movingScalarVolume,
          "anglesNumber": anglesNumber,
          }

    cliRegistrationNode = slicer.cli.run( imageRegistration,None,parameters,wait_for_completion=True )
    angle = cliRegistrationNode.GetParameterDefault(0,2)

    camera = self.cameraForNavigation.GetCamera()
    camera.Roll(float(angle))

    slicer.mrmlScene.RemoveNode(movingScalarVolume)
    slicer.mrmlScene.RemoveNode(realScalarVolume)
