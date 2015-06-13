import os
import unittest
from __main__ import vtk, qt, ctk, slicer
import numpy
import numpy.linalg
from vtk.util.numpy_support import vtk_to_numpy
import csv
import math

#
# Bronchoscopy
#

class Bronchoscopy:
  def __init__(self, parent):
    parent.title = "Bronchoscopy" # TODO make this more human readable by adding spaces
    parent.categories = ["Endoscopy"]
    parent.dependencies = []
    parent.contributors = ["Pietro Nardelli & Alberto Corvo (University College Cork)"] 
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
    self.cameraObserverTag= None

    #
    # Sensor Tracking Variables
    #
    self.sensorTimer = qt.QTimer()
    self.sensorTimer.setInterval(1)
    self.sensorTimer.connect('timeout()', self.ReadPosition)

    self.registrationTimer = qt.QTimer()
    self.registrationTimer.setInterval(5000)
    self.registrationTimer.connect('timeout()', self.registerImage)

    self.checkStreamingTimer = qt.QTimer()
    self.checkStreamingTimer.setInterval(5)
    self.checkStreamingTimer.connect('timeout()', self.showVideoStreaming)

    self.centerlinePointsList = []
    self.fiducialNode = None

    self.pathCreated = 0

    self.pathModelNamesList = []
   
    self.probeCalibrationTransform = None
    self.centerlineCompensationTransform = None
    self.cameraForNavigation = None
    self.cNode = None
    self.probeToTrackerTransformNode = None
    self.videoStreamingNode = None

    self.customLayoutId = 501

    self.layoutManager = slicer.app.layoutManager()
    
    self.setLayout()

    self.firstThreeDView = self.layoutManager.threeDWidget( 0 ).threeDView()
    self.secondThreeDView = self.layoutManager.threeDWidget( 1 ).threeDView()

    self.updateGUI()

    if not parent:
      self.setup()
      self.parent.show()
      self.updateGUI()

  def setLayout(self):
    customLayout = ("<layout type=\"vertical\" split=\"true\" >"
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

  def cleanup(self):
    pass

  def updateGUI(self):
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
    # Reload and Test area
    #
    reloadCollapsibleButton = ctk.ctkCollapsibleButton()
    reloadCollapsibleButton.text = "Reload Section"
    self.layout.addWidget(reloadCollapsibleButton)
    self.layout.setSpacing(6)
    reloadFormLayout = qt.QFormLayout(reloadCollapsibleButton)

    # reload button
    # (use this during development, but remove it when delivering
    #  your module to users)
    self.reloadButton = qt.QPushButton("Reload")
    self.reloadButton.toolTip = "Reload this module."
    self.reloadButton.name = "Bronchoscopy Reload"
    reloadFormLayout.addWidget(self.reloadButton)
    self.reloadButton.connect('clicked()', self.onReload)

    # Instantiate and connect widgets ...

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
    #registrationFormLayout.addRow("Registration Fiducials List: ", self.registrationSelector)

    self.createRegistrationFiducialsButton = qt.QPushButton("Create Registration Points List")
    self.createRegistrationFiducialsButton.toolTip = "Create fiducial list for the registration."
    self.createRegistrationFiducialsButton.setFixedSize(160,35)
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,246,142)")

    regBox.addWidget(self.registrationSelector)
    regBox.addWidget(self.createRegistrationFiducialsButton)

    self.folderPathSelection = qt.QLineEdit()
    self.folderPathSelection.setReadOnly(True)
    #self.folderPathSelection.setFixedWidth(200)

    selectionBox = qt.QHBoxLayout()
    registrationFormLayout.addRow(selectionBox)

    self.selectFolderButton = qt.QPushButton("Select Folder")
    self.selectFolderButton.toolTip = "Select folder where to save the txt file containing the fiducial points."
    self.selectFolderButton.setFixedSize(100,35)
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
    self.labelSelector.nodeTypes = ( ("vtkMRMLScalarVolumeNode"), "" )
    self.labelSelector.addAttribute( "vtkMRMLScalarVolumeNode", "LabelMap", 1 )
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
    self.fiducialsCollapsibleButton.text = "Centerline Fiducials List"
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
    self.fiducialListSelector.setToolTip( "Select centerline fiducial list already uploaded" )
    fiducialFormLayout.addRow("Centerline Fiducials List: ", self.fiducialListSelector)
 
    ########################################################################################################
    #### Optional Push Button To Create A List Of Fiducial Starting From The Extracted Centerline Points ###
    ########################################################################################################

    self.CreateFiducialListButton = qt.QPushButton("Create Fiducial List From Centerline")
    self.CreateFiducialListButton.toolTip = "Create a list of fiducial points starting from the extracted cenetrline bof the 3D model."
    self.CreateFiducialListButton.setFixedSize(240,25)

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
    self.pathCreationCollapsibleButton.enabled = True
    self.layout.addWidget(self.pathCreationCollapsibleButton)
    pathCreationFormLayout = qt.QFormLayout(self.pathCreationCollapsibleButton)

    # Button To Create A Fiducial List Containing All The Points On The ROIs

    self.pointsListSelector = slicer.qMRMLNodeComboBox()
    self.pointsListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.pointsListSelector.selectNodeUponCreation = True
    self.pointsListSelector.addEnabled = True
    self.pointsListSelector.baseName = 'PathFiducials'
    self.pointsListSelector.removeEnabled = True
    self.pointsListSelector.noneEnabled = True
    self.pointsListSelector.showHidden = False
    self.pointsListSelector.showChildNodeTypes = False
    self.pointsListSelector.setMRMLScene( slicer.mrmlScene )
    self.pointsListSelector.setToolTip( "Select points indicating ROIs to reach." )
    self.pointsListSelector.setFixedWidth(180)
    pathCreationFormLayout.addRow("ROI(s) Points List: ", self.pointsListSelector)

    self.createROIFiducialsButton = qt.QPushButton("Add New ROI Point(s)")
    self.createROIFiducialsButton.toolTip = "Add new ROI point(s)."
    self.createROIFiducialsButton.setFixedSize(160,35)

    ROIBox = qt.QHBoxLayout()
    pathCreationFormLayout.addRow(ROIBox)

    ROIBox.addWidget(self.pointsListSelector)
    ROIBox.addWidget(self.createROIFiducialsButton)

    # Button To Create A Fiducial List Containing The Points On The Labels Closest To The ROIs

    self.labelPointsListSelector = slicer.qMRMLNodeComboBox()
    self.labelPointsListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.labelPointsListSelector.selectNodeUponCreation = True
    self.labelPointsListSelector.addEnabled = True
    self.labelPointsListSelector.baseName = 'LabelFiducials'
    self.labelPointsListSelector.removeEnabled = True
    self.labelPointsListSelector.noneEnabled = True
    self.labelPointsListSelector.showHidden = False
    self.labelPointsListSelector.showChildNodeTypes = False
    self.labelPointsListSelector.setMRMLScene( slicer.mrmlScene )
    self.labelPointsListSelector.setToolTip( "Select points on the label closest to the ROI." )
    self.labelPointsListSelector.setFixedWidth(180)
    pathCreationFormLayout.addRow("Label(s) Points List: ", self.labelPointsListSelector)

    self.createLabelsFiducialsButton = qt.QPushButton("Add New Label Point(s)")
    self.createLabelsFiducialsButton.toolTip = "Add point(s) on the closest labels to the ROIs."
    self.createLabelsFiducialsButton.setFixedSize(160,35)

    labelsBox = qt.QHBoxLayout()
    pathCreationFormLayout.addRow(labelsBox)

    labelsBox.addWidget(self.labelPointsListSelector)
    labelsBox.addWidget(self.createLabelsFiducialsButton)

    # Combobox listing all the ROIs points
    self.ROIsPoints = qt.QComboBox()
    self.ROIsPoints.setFixedWidth(180)

    # Button to create new path points
    self.createNewPathPointsButton = qt.QPushButton("Add New Path Point(s)")
    self.createNewPathPointsButton.toolTip = "Add new path point(s) to improve path creation."
    self.createNewPathPointsButton.setFixedSize(160,35)

    ROIPointSelectionBox = qt.QHBoxLayout()
    #pathCreationFormLayout.addRow("New Path Points List: ",)
    pathCreationFormLayout.addRow(ROIPointSelectionBox)

    ROIPointSelectionBox.addWidget(self.ROIsPoints)
    ROIPointSelectionBox.addWidget(self.createNewPathPointsButton)
    
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

    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode() and self.centerlinePointsList != []:
        self.PathCreationButton.enabled = True
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
    else:
        self.PathCreationButton.enabled = False
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")

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

    #############################################################################################
    ###########################  Sensor Tracker Collapsible Button  #############################
    #############################################################################################

    trackerCollapsibleButton = ctk.ctkCollapsibleButton()
    trackerCollapsibleButton.text = "Probe Tracking"
    self.layout.addWidget(trackerCollapsibleButton)
    self.layout.setSpacing(20)
    trackerFormLayout = qt.QVBoxLayout(trackerCollapsibleButton)

    ##############################################################################################
    ###############################  Matlab/Probe Track Button  ##################################
    ##############################################################################################
    self.ProbeTrackButton = qt.QPushButton("Track Sensor")
    self.ProbeTrackButton.toolTip = "Track sensor output."
    self.ProbeTrackButton.setFixedSize(250,60)
    #self.ProbeTrackButton.setFixedHeight(40)
    self.ProbeTrackButton.checkable = True
   
    trackerFormLayout.addWidget(self.ProbeTrackButton, 0, 4)

    ##############################################################################################
    ##################################  Reset Camera Button  #####################################
    ##############################################################################################

    self.ResetCameraButton = qt.QPushButton("Reset Camera")

    self.ResetCameraButton.toolTip = "Reset camera if moved away."
    self.ResetCameraButton.setFixedSize(100,50)
    self.ResetCameraButton.enabled = False
    
    trackerFormLayout.addWidget(self.ResetCameraButton, 0, 4)

    # Enable ProbeTracKButton
    if self.fiducialListSelector.currentNode() or self.centerlinePointsList != []:
        self.ProbeTrackButton.enabled = True
    else:
        self.ProbeTrackButton.enabled = False


    ########################################################################################
    ################################ Image Registration ####################################
    ########################################################################################
    imgRegCollapsibleButton = ctk.ctkCollapsibleButton()
    imgRegCollapsibleButton.text = "Image Registration Section"
    self.layout.addWidget(imgRegCollapsibleButton)
    self.layout.setSpacing(20)
    imgRegFormLayout = qt.QFormLayout(imgRegCollapsibleButton)

    realImgSelectionBox = qt.QHBoxLayout()
    imgRegFormLayout.addRow(realImgSelectionBox)

    self.ImageRegistrationButton = qt.QPushButton("Start Image Registration")
    self.ImageRegistrationButton.toolTip = "Start registration between real and virtual images."
    self.ImageRegistrationButton.setFixedSize(250,50)
    self.ImageRegistrationButton.enabled = True
    self.ImageRegistrationButton.checkable = True
    
    imgRegButtonBox = qt.QVBoxLayout()
    imgRegFormLayout.addRow(imgRegButtonBox)

    imgRegButtonBox.addWidget(self.ImageRegistrationButton, 0, 4)


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
    self.VideoRegistrationButton.setFixedSize(250,50)
    self.VideoRegistrationButton.enabled = True
    self.VideoRegistrationButton.checkable = True
    
    VSButtonBox = qt.QVBoxLayout()
    videoStreamingFormLayout.addRow(VSButtonBox)

    VSButtonBox.addWidget(self.VideoRegistrationButton, 0, 4)

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
    self.CreateFiducialListButton.connect('clicked(bool)',self.onCreateAndSaveFiducialList)

    self.pointsListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
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
    self.ResetCameraButton.connect('clicked(bool)',self.onResetCameraButtonPressed)

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

    if self.registrationSelector.currentNode() and self.folderPathSelection.text:
      self.RegFidListButton.enabled = True
    else:
      self.RegFidListButton.enabled = False

    if self.inputSelector.currentNode() and self.labelSelector.currentNode():
      inputVolume = self.inputSelector.currentNode()
      modelDisplayNode = inputVolume.GetDisplayNode()
      modelDisplayNode.SetColor(1.0, 0.8, 0.7)
      modelDisplayNode.SetFrontfaceCulling(1)
      modelDisplayNode.SetBackfaceCulling(0)

      self.ExtractCenterlineButton.enabled = True
      self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(175,255,253)")
   
      if self.fiducialListSelector.currentNode() or self.centerlinePointsList != []:
        self.ProbeTrackButton.enabled = True
      else:
        self.ProbeTrackButton.enabled = False
        self.ResetCameraButton.enabled = False
        #self.ImageRegistrationButton.enabled = False
    else:
      self.ExtractCenterlineButton.enabled = False
      self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.PathCreationButton.enabled = False
      self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.ProbeTrackButton.enabled = False
      self.ResetCameraButton.enabled = False
      #self.ImageRegistrationButton.enabled = False

    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode() and self.centerlinePointsList != []:
       self.PathCreationButton.enabled = True
       self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
    else:
       self.PathCreationButton.enabled = False
       self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")

    if self.centerlinePointsList != []:
      self.CreateFiducialListButton.enabled = True

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
    self.ResetCameraButton.enabled = False
    #self.ImageRegistrationButton.enabled = False

    self.inputSelector.enabled = False
    self.labelSelector.enabled = False
    self.fiducialListSelector.enabled = False
    self.pointsListSelector.enabled = False
    self.registrationSelector.enabled = False

  def enableSelectors(self):

    self.selectFolderButton.enabled = True
    self.selectFolderButton.setStyleSheet("background-color: rgb(255,246,142)")

    self.createRegistrationFiducialsButton.enabled = True
    self.createRegistrationFiducialsButton.setStyleSheet("background-color: rgb(255,246,142)")

    self.inputSelector.enabled = True
    self.labelSelector.enabled = True
    self.fiducialListSelector.enabled = True
    self.pointsListSelector.enabled = True
    self.registrationSelector.enabled = True

  def onReload(self,moduleName="Bronchoscopy"):
    """Generic reload method for any scripted module.
    ModuleWizard will subsitute correct default moduleName.
    """
    import os
    import unittest
    from __main__ import vtk, qt, ctk, slicer, numpy
    import imp, sys

    widgetName = moduleName + "Widget"

    # reload the source code
    # - set source file path
    # - load the module to the global space
    filePath = eval('slicer.modules.%s.path' % moduleName.lower())
    p = os.path.dirname(filePath)
    if not sys.path.__contains__(p):
      sys.path.insert(0,p)
    fp = open(filePath, "r")
    globals()[moduleName] = imp.load_module(
        moduleName, fp, filePath, ('.py', 'r', imp.PY_SOURCE))
    fp.close()

    # rebuild the widget
    # - find and hide the existing widget
    # - create a new widget in the existing parent
    parent = slicer.util.findChildren(name='%s Reload' % moduleName)[0].parent().parent()
    for child in parent.children():
      try:
        child.hide()
      except AttributeError:
        pass
    # Remove spacer items
    item = parent.layout().itemAt(0)
    while item:
      parent.layout().removeItem(item)
      item = parent.layout().itemAt(0)

    # delete the old widget instance
    if hasattr(globals()['slicer'].modules, widgetName):
      getattr(globals()['slicer'].modules, widgetName).cleanup()

    # create new widget inside existing parent
    globals()[widgetName.lower()] = eval(
        'globals()["%s"].%s(parent)' % (moduleName, widgetName))
    globals()[widgetName.lower()].setup()
    setattr(globals()['slicer'].modules, widgetName, globals()[widgetName.lower()])

##################################################################################################
############################## REGISTRATION FIDUCIALS SAVING ##################################### 
##################################################################################################
# Copy fiducials used for the registration within a txt file to be saved in the Matlab folder

  def onCreateRegFidList(self):

    self.disableButtonsAndSelectors()
    markupsList = slicer.vtkMRMLMarkupsFiducialNode()
    markupsList.SetName('RegistrationPoints')
    slicer.mrmlScene.AddNode(markupsList)
    displayNode = markupsList.GetDisplayNode()
    displayNode.SetSelectedColor(0.0,0.0,1.0)

    self.registrationSelector.setCurrentNodeID(markupsList.GetID())

    self.enableSelectors()
    self.onSelect()

  def onSaveRegistrationPoints(self):
    
    self.disableButtonsAndSelectors()
   
    regFidListNode = self.registrationSelector.currentNode()
    point = [0,0,0]
    pointsList = []
    for i in xrange(regFidListNode.GetNumberOfFiducials()):
      regFidListNode.GetNthFiducialPosition(i,point)
      p = [point[0],point[1],point[2]]
      pointsList.append(p)

    localDirectory = self.folderPathSelection.text + "/RegistrationPoints.csv"

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

    if self.fiducialListSelector.currentNode():  # if a fiducial list was already uploaded, all that follows is not necessary!
      self.fiducialNode = self.fiducialListSelector.currentNode()
    else:
      self.centerline = slicer.vtkMRMLScalarVolumeNode()
      #self.centerline.addAttribute( "vtkMRMLScalarVolumeNode", "LabelMap", 1 )
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
      self.Smoothing(centerlinePolydata, self.points, iterations)

    self.ProbeTrackButton.enabled = True

    if self.fiducialNode:
      disNode = self.fiducialNode.GetDisplayNode()
      disNode.SetVisibility(0)
      for i in xrange(self.fiducialNode.GetNumberOfFiducials()):
        point = [0,0,0]
        self.fiducialNode.GetNthFiducialPosition(i,point)
        self.centerlinePointsList.append(point)

    slicer.mrmlScene.RemoveNode(self.fiducialListSelector.currentNode())

    return True

  def Smoothing(self, centModel, modelPoints, iterationsNumber):
    
    NumberOfCells = centModel.GetNumberOfCells()

    pointsList = []
    distancePointsAbove = []
    distancePointsBelow = []

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

    localDirectory = "C:/Users/Lab/Desktop/CenterlineFiducials.fcsv"
    a =[]
    with open(localDirectory, "wb") as f:
      writer = csv.writer(f,)
      version = slicer.app.applicationVersion
      firstRow = '# Markups fiducial file version = ' + str(version[0:3])
      a.append(firstRow)
      writer.writerow(a)
      writer.writerow(['# CoordinateSystem = 0'])
      writer.writerow(['# columns = id']+['x']+['y']+['z']+['ow']+['ox']+['oy']+['oz']+['vis']+['sel']+['lock']+['label']+['desc']+['associatedNodeID'])
      writer.writerows(fiducialList)  
   
    self.enableSelectors()

    self.onSelect()


#######################################################################################################
##################################### PATH CREATION AND INFO ########################################## 
#######################################################################################################
    
  def onCreateROIFiducialsList(self):
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
    displayNode.SetGlyphScale(6)
    displayNode.SetTextScale(1)

    markupLogic.SetActiveListID(markupsList)
    self.pointsListSelector.setCurrentNode(markupsList)

    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.SwitchToPersistentPlaceMode()

    #self.updateGUI()
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
    self.updateComboBox()
    self.pendingUpdate = False
    self.updatingFiducials = False

  def updateComboBox(self):
    '''Update the ROIs combobox'''
    fiducialsLogic = slicer.modules.markups.logic()
    activeListID = fiducialsLogic.GetActiveListID()
    activeList = slicer.util.getNode(activeListID)

    if activeList:
      if activeList.GetNumberOfFiducials() > 0:
        lastElement = activeList.GetNumberOfFiducials() - 1
        self.ROIsPoints.addItem(activeList.GetNthFiducialLabel(lastElement))

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

    if self.addNewPathPoints:
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

        AddedPathPointsList.AddFiducial(fidPosition[0],fidPosition[1],fidPosition[2])

      markupLogic = slicer.modules.markups.logic()
      markupLogic.SetActiveListID(markupsList)
      self.labelPointsListSelector.setCurrentNode(markupsList)
    
  def onCreateLabelsFiducialsList(self):
    
    self.fitSlicesToBackground()

    LabelPointFiducialList = slicer.util.getNode('LabelsPoints')
    markupLogic = slicer.modules.markups.logic()

    if LabelPointFiducialList:
      markupsList = LabelPointFiducialList
    else:
      markupsList = slicer.vtkMRMLMarkupsFiducialNode()
      markupsList.SetName('LabelsPoints')
      slicer.mrmlScene.AddNode(markupsList)

    fidNode =  slicer.util.getNode('LabelsPoints')
    fidDisplayNode = fidNode.GetDisplayNode()
    fidDisplayNode.SetGlyphScale(3)
    fidDisplayNode.SetTextScale(0)
    fidDisplayNode.SetSelectedColor(0.0,1.0,1.0)
    
    markupLogic.SetActiveListID(markupsList)
    self.labelPointsListSelector.setCurrentNode(markupsList)

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

    self.fitSlicesToBackground()
    
    self.addNewPathPoints = True
    appLogic = slicer.app.applicationLogic()
    selectionNode = appLogic.GetSelectionNode()
    selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
    interactionNode = appLogic.GetInteractionNode()
    interactionNode.SwitchToPersistentPlaceMode()

    self.showSelectedROI()
    
    #self.updateGUI()
  def onDefaultLayoutButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(self.customLayoutId)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if ROIsList:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()
      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()
      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic() 

  def onRedViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(6)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if ROIsList:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()
      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()
      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic() 
 
  def onYellowViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(7)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if ROIsList:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()
      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()
      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic() 

  def onGreenViewButton(self):
    self.fitSlicesToBackground()
    self.layoutManager.setLayout(8)

    fidIndex = self.ROIsPoints.currentIndex
    ROIsList = slicer.util.getNode('ROIFiducials')
    if ROIsList:
      fidPosition = [0,0,0]
      ROIsList.GetNthFiducialPosition(fidIndex, fidPosition)

      yellowWidget = self.layoutManager.sliceWidget('Yellow')
      yellowLogic = yellowWidget.sliceLogic()
      greenWidget = self.layoutManager.sliceWidget('Green')
      greenLogic = greenWidget.sliceLogic()
      redWidget = self.layoutManager.sliceWidget('Red')
      redLogic = redWidget.sliceLogic() 

  def onPathCreationButton(self):
    fiducials = slicer.util.getNode('LabelsPoints')

    if fiducials:
      self.disableButtonsAndSelectors()

      # Create Centerline Path   
      if self.centerlinePointsList != []:
        self.CreateFiducialListButton.enabled = True
      pos = [0,0,0]
      targetPos = [0,0,0]
      for i in xrange(self.ROIsPoints.count):
        fiducials.GetNthFiducialPosition(i,targetPos)
        firstPath = self.pathComputation(self.inputSelector.currentNode(), targetPos)
        
        listName = 'AddedPathPointsList-' + str(i+1)
        AddedPathPointsList = slicer.util.getNode(listName)

        if AddedPathPointsList:
          if AddedPathPointsList.GetNumberOfFiducials() > 0:
            firstPath.GetPoint(0,targetPos)
            AddedPathPointsList.AddFiducial(targetPos[0],targetPos[1],targetPos[2])
            computedPath = self.computeAddedPath(AddedPathPointsList)
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
        modelDisplay.SetSliceIntersectionThickness(5)
        slicer.mrmlScene.AddNode(modelDisplay)
        model.SetAndObserveDisplayNodeID(modelDisplay.GetID())

        # Add to scene
        if vtk.VTK_MAJOR_VERSION <= 5:
          # shall not be needed.
          modelDisplay.SetInputPolyData(model.GetPolyData())
        slicer.mrmlScene.AddNode(model)

        self.pathModelNamesList.append(model.GetName()) # Save names to delete models before creating the new ones.
      
      self.pathCreated = 1
      
      slicer.mrmlScene.RemoveNode(fiducials)
      ROINode = slicer.util.getNode('ROIFiducials')

      for i in xrange(ROINode.GetNumberOfFiducials()):
        name = 'AddedPathPointsList-'+str(i+1)
        node = slicer.util.getNode(name)
        slicer.mrmlScene.RemoveNode(node)

      self.enableSelectors()
      self.onSelect()

    else:
      string = 'The selected path fiducial list contains ' + str(fiducials.GetNumberOfFiducials()) + ' fiducials. Number of fiducials in the list must be >= 1.'
      raise Exception(string)
   
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

    if len(self.pathModelNamesList) > 0:
      for n in xrange(len(self.pathModelNamesList)):
        name = self.pathModelNamesList[n]
        model = slicer.util.getNode(name)
        slicer.mrmlScene.RemoveNode(model)
      self.pathModelNamesList = []
        
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


  def computeAddedPath(self, fiducialListNode, dl=0.5):

    self.dl = dl # desired world space step size (in mm)
    self.dt = dl # current guess of parametric stepsize
    self.fids = fiducialListNode

    # hermite interpolation functions
    self.h00 = lambda t: 2*t**3 - 3*t**2     + 1
    self.h10 = lambda t:   t**3 - 2*t**2 + t
    self.h01 = lambda t:-2*t**3 + 3*t**2
    self.h11 = lambda t:   t**3 -   t**2

    # n is the number of control points in the piecewise curve

    if self.fids.GetClassName() == "vtkMRMLAnnotationHierarchyNode":
      # slicer4 style hierarchy nodes
      collection = vtk.vtkCollection()
      self.fids.GetChildrenDisplayableNodes(collection)
      self.n = collection.GetNumberOfItems()
      if self.n == 0:
        return
      self.p = numpy.zeros((self.n,3))
      for i in xrange(self.n):
        f = collection.GetItemAsObject(i)
        coords = [0,0,0]
        f.GetFiducialCoordinates(coords)
        self.p[i] = coords
    elif self.fids.GetClassName() == "vtkMRMLMarkupsFiducialNode":
      # slicer4 Markups node
      self.n = self.fids.GetNumberOfFiducials()
      n = self.n
      if n == 0:
        return
      # get fiducial positions
      # sets self.p
      self.p = numpy.zeros((n,3))
      for i in xrange(n):
        coord = [0.0, 0.0, 0.0]
        self.fids.GetNthFiducialPosition(i, coord)
        self.p[i] = coord
    else:
      # slicer3 style fiducial lists
      self.n = self.fids.GetNumberOfFiducials()
      n = self.n
      if n == 0:
        return
      # get control point data
      # sets self.p
      self.p = numpy.zeros((n,3))
      for i in xrange(n):
        self.p[i] = self.fids.GetNthFiducialXYZ(i)

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
      print len(self.centerlinePointsList)
      pathPolydata = pathModel.GetPolyData()
      pathPoints = pathPolydata.GetPoints()
      for j in xrange(pathPoints.GetNumberOfPoints()):
        self.centerlinePointsList.append(pathPoints.GetPoint(j))
      print len(self.centerlinePointsList)

    ###### Create a list of points from the vtkPoints object ############
    if self.centerlinePointsList != []:

      for i in xrange(len(self.centerlinePointsList)):
        point = self.centerlinePointsList[i]
        p = [point[0],point[1],point[2]]
        self.centerlinePointsList.append(p)

      # Avoid repetition of the same point twice
      self.centerlinePointsList = numpy.array([list(x) for x in set(tuple(x) for x in self.centerlinePointsList)])
      print len(self.centerlinePointsList)
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
      self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,156,126)")

      self.disableButtonsAndSelectors()
      self.ProbeTrackButton.enabled = True
      self.ResetCameraButton.enabled = True

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

      if self.centerlineCompensationTransform == None:
        centerlineCompensationTransformNodes = slicer.mrmlScene.GetNodesByName('centerlineCompensationTransform')
        if centerlineCompensationTransformNodes.GetNumberOfItems() == 0:
          self.centerlineCompensationTransform = slicer.vtkMRMLLinearTransformNode()
          self.centerlineCompensationTransform.SetName('centerlineCompensationTransform')
          slicer.mrmlScene.AddNode(self.centerlineCompensationTransform)
        else:
          self.centerlineCompensationTransform = centerlineCompensationTransformNodes.GetItemAsObject(0)

      probeNode = slicer.util.getNode('ProbeModel')
      if probeNode:
        probeDisplayNode = probeNode.GetDisplayNode()
        probeDisplayNode.SetColor(0, 0, 1)
        probeDisplayNode.SetSliceIntersectionVisibility(1)
        probeDisplayNode.SetSliceIntersectionThickness(4)

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

      ################## Camera 1 is connected to the transform #####################
         
      self.onResetCameraButtonPressed()
      self.cameraForNavigation.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
      
      ####################### Set clipping range for the first camera ####################
      camera = self.cameraForNavigation.GetCamera()
      camera.SetClippingRange(0.7081381565016212, 708.1381565016211) # to be checked

      ########## Camera 2 is will automatically follow the probe ##########
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
       
    else:  # When button is released...      
      self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.ResetCameraButton.enabled = False
      #self.ImageRegistrationButton.enabled = False

      self.sensorTimer.stop()
      self.registrationTimer.stop()
      self.cNode.Stop()
      #self.cNode = None
      #self.cameraForNavigation = None
      #self.probeCalibrationTransform = None

      lastFPBeforeStoppingTracking = [0,0,0]
      lastPosBeforeStoppingTracking = [0,0,0]
      lastViewUp = [0,0,0]

      self.cameraForNavigation.GetFocalPoint(lastFPBeforeStoppingTracking)
      self.cameraForNavigation.GetPosition(lastPosBeforeStoppingTracking)
      self.cameraForNavigation.GetViewUp(lastViewUp)

      self.enableSelectors()
      self.onSelect()

      self.cameraForNavigation.SetFocalPoint(lastFPBeforeStoppingTracking[0],lastFPBeforeStoppingTracking[1],lastFPBeforeStoppingTracking[2])
      self.cameraForNavigation.SetPosition(lastPosBeforeStoppingTracking[0],lastPosBeforeStoppingTracking[1],lastPosBeforeStoppingTracking[2])
      camera = self.cameraForNavigation.GetCamera()  
      self.cameraForNavigation.SetViewUp(lastViewUp)
      camera.SetClippingRange(0.7081381565016212, 708.1381565016211) # to be checked

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

          if self.probeCalibrationTransform.GetTransformNodeID() == None:
            self.probeCalibrationTransform.SetAndObserveTransformNodeID(self.centerlineCompensationTransform.GetID())

          self.CheckCurrentPosition(transformMatrix)
  
  def onResetCameraButtonPressed(self):
    cameraNodes = slicer.mrmlScene.GetNodesByName('Default Scene Camera')
    self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
    if cameraNodes.GetNumberOfItems() > 0:
      if self.cameraForNavigation.GetTransformNodeID() == None:
        self.cameraForNavigation.SetPosition(-1.0,0.0,0.0)
        self.cameraForNavigation.SetFocalPoint(-5.0,0.0,.0)
        viewUp = [0.0,0.0,-1.0]
        self.cameraForNavigation.SetViewUp(viewUp)
      else:
        tNode = slicer.util.getNode('centerlineCompensationTransform')
        if tNode:
          transformMatrix = vtk.vtkMatrix4x4()
          tNode.GetMatrixTransformToParent(transformMatrix)
          pos = [0,0,0]
          pos[0] = transformMatrix.GetElement(0,3)
          pos[1] = transformMatrix.GetElement(1,3)
          pos[2] = transformMatrix.GetElement(2,3)
          self.cameraForNavigation.SetPosition(pos[0],pos[1],pos[2])
          camera = self.cameraForNavigation.GetCamera()
          vpn = camera.GetViewPlaneNormal()
          pos = numpy.asarray(pos)
          vpn=numpy.asarray(vpn)
          FP = pos-4*vpn
          #self.cameraForNavigation.GetFocalPoint(FP)
          self.cameraForNavigation.SetFocalPoint(FP[0],FP[1],FP[2])

      self.cameraForNavigation.SetViewAngle(55)
      
  def CheckCurrentPosition(self, tMatrix):

    distance = []

    '''if self.fiducialNode == None:
      fiducialNodesCollection = slicer.mrmlScene.GetNodesByName('CenterlineFiducials')
      self.fiducialNode = fiducialNodesCollection.GetItemAsObject(0)'''

    originalCoord = [0,0,0]
    originalCoord[0] = tMatrix.GetElement(0,3)
    originalCoord[1] = tMatrix.GetElement(1,3)
    originalCoord[2] = tMatrix.GetElement(2,3)

    originalCoord   = numpy.asarray(originalCoord)
    self.centerlinePointsList = numpy.asarray(self.centerlinePointsList)

    distance = ((self.centerlinePointsList-originalCoord)**2).sum(axis=1)
    ndx = distance.argsort()
    closestPoint = self.centerlinePointsList[ndx[0]]

    tMatrix.SetElement(0,3,closestPoint[0])
    tMatrix.SetElement(1,3,closestPoint[1])
    tMatrix.SetElement(2,3,closestPoint[2])

    self.centerlinePointsList = self.centerlinePointsList.tolist()

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

    pos = [0,0,0]
    self.secondCamera.SetFocalPoint(x,y,z)
    self.secondCamera.SetPosition(x,y+250,z)

    # force the camera position to be a bit higher to better watch the path
    self.cameraForNavigation.SetPosition(x,y,z-1)
    camera=self.cameraForNavigation.GetCamera()
    '''p = camera.GetPosition()
    vpn = camera.GetViewPlaneNormal()
    print "position: ",p
    print "VPN: ", vpn
    p=numpy.asarray(p)
    vpn=numpy.asarray(vpn)
    theorFP = p-4*vpn
    print "Theoretical FP: ", theorFP
    print "real prev FP: ", camera.GetFocalPoint()
    d=[0,0,0]
    #self.cameraForNavigation.SetFocalPoint(theorFP)
    self.cameraForNavigation.GetFocalPoint(d)
    print "real new FP: ", d'''  
    if len(self.pathModelNamesList) > 0:
      pathModel = self.pathModelSelector.currentNode()
      pathPolyData = pathModel.GetPolyData()
      self.distanceToTargetComputation(pathPolyData, closestPoint)

  def distanceToTargetComputation(self, polyData, secondPoint):

    numberOfPoints = polyData.GetNumberOfPoints()
    
    firstPoint = [0,0,0]
    polyData.GetPoint(numberOfPoints-1, firstPoint)

    squaredDistance = vtk.vtkMath.Distance2BetweenPoints(firstPoint, secondPoint)
    length = math.sqrt(squaredDistance)
    length = int(length)
    
    # Change color of the fiducial when close to the ROI
    ROIFiducialList = slicer.util.getNode('ROIFiducials')
    displayNode = ROIFiducialList.GetDisplayNode()
    if length <= 3:
      displayNode.SetSelectedColor(0.4, 1.0, 1.0)
    else:
      displayNode.SetSelectedColor(1.0,0.0,0.0)      
    
    string_length = str(length) + ' mm'        
       
    self.distanceToTarget.setText(string_length)
    
    distToTarget = 'Distance To Target: ' + string_length

    self.firstViewCornerAnnotation.SetText(1,distToTarget)
    self.secondViewCornerAnnotation.SetText(1,distToTarget)
    
    color = qt.QColor('yellow')
    firsTxtProperty = self.firstViewCornerAnnotation.GetTextProperty()
    firsTxtProperty.SetColor(color.redF(), color.greenF(), color.blueF())
    firsTxtProperty.SetBold(1)
    #firsTxtProperty.SetFontFamilyAsString('Courier')
    
    secondTxtProperty = self.secondViewCornerAnnotation.GetTextProperty()
    secondTxtProperty.SetColor(color.redF(), color.greenF(), color.blueF())
    secondTxtProperty.SetBold(1)
    #secondTxtProperty.SetFontFamilyAsString('Courier')
    
    self.secondThreeDView.forceRender()

  ###########################################################################################
  ################################## Image Registration #####################################
  ###########################################################################################
  def onStartImageRegistrationButtonPressed(self, checked):
    if checked:
      self.registrationTimer.start()
      self.ImageRegistrationButton.text = "Stop Image Registration"
    else:
      self.registrationTimer.stop()
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
    
    rw = self.firstThreeDView.renderWindow()
    wti = vtk.vtkWindowToImageFilter()
    wti.SetInput(rw)
    displayNode.SetVisibility(0)
    slicer.app.processEvents()
    wti.Update()
    displayNode.SetVisibility(1)
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
    angle = int(angle)

    self.firstThreeDView.rollDirection = self.firstThreeDView.RollRight
    self.firstThreeDView.pitchRollYawIncrement = abs(angle)
    self.firstThreeDView.roll()

    slicer.mrmlScene.RemoveNode(movingScalarVolume)
    slicer.mrmlScene.RemoveNode(realScalarVolume)

  def startVideoStreaming(self, checked):
    if checked:
      self.VideoRegistrationButton.setText("Stop Video Streaming")
      if self.videoStreamingNode == None:
        streamingNodes = slicer.mrmlScene.GetNodesByName('streamingConnector')
        if streamingNodes.GetNumberOfItems() == 0:
          self.videoStreamingNode = slicer.vtkMRMLIGTLConnectorNode()
          slicer.mrmlScene.AddNode(self.videoStreamingNode)
          self.videoStreamingNode.SetName('streamingConnector')
        else:
          self.videoStreamingNode = streamingNodes.GetItemAsObject(0)

        #self.videoStreamingNode.SetType(2)
        self.videoStreamingNode.SetTypeClient('localhost',18945)
        self.videoStreamingNode.Start()

        self.checkStreamingTimer.start()
    else:
      if self.videoStreamingNode != None:
        self.VideoRegistrationButton.setText("Start Video Streaming")
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
