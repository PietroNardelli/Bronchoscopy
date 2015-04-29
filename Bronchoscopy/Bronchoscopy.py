import os
import unittest
from __main__ import vtk, qt, ctk, slicer
import numpy

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
    Scripted loadable module bundled in an extension for centerline extraction and virtual navigation within a 3D airway model.
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

    self.points = vtk.vtkPoints()
    self.pointsList = []
    self.fiducialNode = None

    self.pathCreated = 0

    self.pathModelNamesList = []
    self.centerlinePoints = vtk.vtkPoints()
    
    self.probeCalibrationTransform = None
    self.centerlineCompensationTransform = None
    self.cameraForNavigation = None
    self.cNode = None
    self.probeToTrackerTransformNode = None

    self.updateGUI()

    if not parent:
      self.setup()
      self.parent.show()
      self.updateGUI()

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

    self.createRegistrationFiducialsButton = qt.QPushButton("Create Registration Points")
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
    self.CreateFiducialListButton.checkable = True

    if self.points.GetNumberOfPoints() > 0:
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
    ######################  Create Path Towards An ROI Section  ########################
    ####################################################################################
    self.pathCreationCollapsibleButton = ctk.ctkCollapsibleButton()
    self.pathCreationCollapsibleButton.text = "Path Creation"
    self.pathCreationCollapsibleButton.setChecked(True)
    self.pathCreationCollapsibleButton.enabled = True
    self.layout.addWidget(self.pathCreationCollapsibleButton)
    pathCreationFormLayout = qt.QFormLayout(self.pathCreationCollapsibleButton)

    self.pointsListSelector = slicer.qMRMLNodeComboBox()
    self.pointsListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.pointsListSelector.selectNodeUponCreation = True
    self.pointsListSelector.addEnabled = True
    self.pointsListSelector.baseName = 'PathFiducial'
    self.pointsListSelector.removeEnabled = True
    self.pointsListSelector.noneEnabled = True
    self.pointsListSelector.showHidden = False
    self.pointsListSelector.showChildNodeTypes = False
    self.pointsListSelector.setMRMLScene( slicer.mrmlScene )
    self.pointsListSelector.setToolTip( "Select points for path creation." )
    pathCreationFormLayout.addRow("Points List: ", self.pointsListSelector)
    
    ###################################################################################
    #############################  Path Creation Button  ############################## 
    ###################################################################################
    self.PathCreationButton = qt.QPushButton("Create Path(s)")
    self.PathCreationButton.toolTip = "Run the algorithm to create the path between the specified points."
    self.PathCreationButton.setFixedSize(300,50)

    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode() and self.points.GetNumberOfPoints()>0:
        self.PathCreationButton.enabled = True
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
    else:
        self.PathCreationButton.enabled = False
        self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")

    bLayout = qt.QVBoxLayout()

    pathCreationFormLayout.addRow(bLayout)
    bLayout.addWidget(self.PathCreationButton,0,4)

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

    if self.fiducialListSelector.currentNode() or self.points.GetNumberOfPoints()>0:
        self.ProbeTrackButton.enabled = True
    else:
        self.ProbeTrackButton.enabled = False

    ########################################################################################
    ################################ Create Connections ####################################
    ########################################################################################

    self.registrationSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.RegFidListButton.connect('clicked(bool)', self.onRegFidListButton)
    self.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.labelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.ExtractCenterlineButton.connect('clicked(bool)', self.onExtractCenterlineButton)
    self.fiducialListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.pointsListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.PathCreationButton.connect('clicked(bool)', self.onPathCreationButton)
    self.ProbeTrackButton.connect('toggled(bool)', self.onProbeTrackButtonToggled)
    self.ResetCameraButton.connect('clicked(bool)',self.onResetCameraButtonPressed)
    self.CreateFiducialListButton.connect('toggled(bool)',self.onCreateFiducialListToggled)
    self.selectFolderButton.connect('clicked(bool)', self.onSelectFolderButton)
    self.createRegistrationFiducialsButton.connect('clicked(bool)', self.onCreateRegFidList)
    
    #
    # Add Vertical Spacer
    #
    self.layout.addStretch(1)
    
    #
    # Update the 3D Views
    #
    self.updateGUI()

  def cleanup(self):
    pass

  def updateGUI(self):
    lm=slicer.app.layoutManager()
    lm.setLayout(15)
    firstThreeDView = lm.threeDWidget( 0 ).threeDView()
    firstThreeDView.resetFocalPoint()
    firstThreeDView.lookFromViewAxis(ctk.ctkAxesWidget().Anterior)
    secondThreeDView = lm.threeDWidget( 1 ).threeDView()
    secondThreeDView.resetFocalPoint()
    secondThreeDView.lookFromViewAxis(ctk.ctkAxesWidget().Anterior)

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
   
      if self.fiducialListSelector.currentNode() or self.points.GetNumberOfPoints()>0:
        self.ProbeTrackButton.enabled = True
      else:
        self.ProbeTrackButton.enabled = False
        self.ResetCameraButton.enabled = False
    else:
      self.ExtractCenterlineButton.enabled = False
      self.ExtractCenterlineButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.PathCreationButton.enabled = False
      self.PathCreationButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.ProbeTrackButton.enabled = False
      self.ResetCameraButton.enabled = False

    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode() and self.points.GetNumberOfPoints()>0:
       self.PathCreationButton.enabled = True
       self.PathCreationButton.setStyleSheet("background-color: rgb(255,246,142)")
    else:
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
    self.ResetCameraButton.enabled = False

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
    displayNode.SetSelectedColor((0,0,255))

    self.registrationSelector.setCurrentNodeID(markupsList.GetID())

    self.enableSelectors()
    self.onSelect()

  def onRegFidListButton(self):
    
    self.disableButtonsAndSelectors()
   
    regFidListNode = self.registrationSelector.currentNode()
    point = [0,0,0]
    pointsList = []
    for i in xrange(regFidListNode.GetNumberOfFiducials()):
      regFidListNode.GetNthFiducialPosition(i,point)
      p = [point[0],point[1],point[2]]
      pointsList.append(p)

    wheretosave = self.folderPathSelection.text + "/RegistrationPoints.txt" 
    numpy.savetxt(wheretosave,pointsList)

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

    if self.points.GetNumberOfPoints() > 0:
      self.CreateFiducialListButton.enabled = True
  
    if self.fiducialNode:
      FList = slicer.mrmlScene.GetNodesByName('F')
      AirwayFiducialList = slicer.mrmlScene.GetNodesByName('AirwayFiducial')
      PathFiducialList = slicer.mrmlScene.GetNodesByName('PathFiducial')
      markupLogic = slicer.modules.markups.logic()
      if FList.GetNumberOfItems() > 0:  
        markupsList = FList.GetItemAsObject(0)
      elif AirwayFiducialList.GetNumberOfItems() > 0:
        markupsList = AirwayFiducialList.GetItemAsObject(0)
      elif PathFiducialList.GetNumberOfItems() > 0:
        markupsList = PathFiducialList.GetItemAsObject(0)
      else:
        markupsList = slicer.vtkMRMLMarkupsFiducialNode()
        markupsList.SetName('PathFiducial')
        slicer.mrmlScene.AddNode(markupsList)
      
      markupLogic.SetActiveListID(markupsList)      

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

      # create fiducial list
      #self.fiducialNode = slicer.vtkMRMLMarkupsFiducialNode()
      #self.fiducialNode.SetName('CenterlineFiducials')
      #slicer.mrmlScene.AddNode(self.fiducialNode)

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
        self.points.InsertNextPoint(point)
   
    self.centerlinePoints.DeepCopy(self.points)
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

#######################################################################################################
####################### Create A Fiducial List With A Fiducial On Each Point ##########################  
#######################################################################################################

  def onCreateFiducialListToggled(self, checked):
    if checked:      
      self.disableButtonsAndSelectors()
      self.CreateFiducialListButton.checked = False

      fNode = slicer.vtkMRMLMarkupsFiducialNode()
      fNode.SetName('CenterlineFiducials')
      slicer.mrmlScene.AddNode(fNode)
      point = [0,0,0]
      for i in xrange(self.points.GetNumberOfPoints()):
        self.points.GetPoint(i,point)
        fNode.AddFiducial(point[0],point[1],point[2])

      dNode = fNode.GetDisplayNode()
      dNode.SetVisibility(0)       
   
      self.enableSelectors()

      self.onSelect()


#######################################################################################################
########################################## PATH CREATION ############################################## 
#######################################################################################################

  def onPathCreationButton(self):
    fiducials = self.pointsListSelector.currentNode()

    if fiducials.GetNumberOfFiducials() > 1 and fiducials.GetNumberOfFiducials() < 10:
      self.pathCreated = 1

      self.disableButtonsAndSelectors()

      # Create Centerline Path 
      self.pathComputation(self.inputSelector.currentNode(), self.pointsListSelector.currentNode()) 
    
      self.enableSelectors()
      self.onSelect()

      if self.points.GetNumberOfPoints() > 0:
        self.CreateFiducialListButton.enabled = True

    else:
      string = 'The selected path fiducial list contains ' + str(fiducials.GetNumberOfFiducials()) + ' fiducials. Number of fiducials in the list must be between 2 and 10.'
      raise Exception(string)
   
    # Update GUI
    self.updateGUI()

  def pathComputation(self, inputModel, STpoints):
    """
    Run the actual algorithm to create the path between the 2 fiducials
    """
    import vtkSlicerVMTKFunctionalitiesModuleLogic

    if len(self.pathModelNamesList) > 0:
      self.points = self.centerlinePoints
      for n in xrange(len(self.pathModelNamesList)):
        name = self.pathModelNamesList[n]
        modelNode = slicer.mrmlScene.GetNodesByName(name)
        model = modelNode.GetItemAsObject(0)
        slicer.mrmlScene.RemoveNode(model)
      self.pathModelNamesList = []
        
    inputPolyData = inputModel.GetPolyData()

    sourcePosition = [0,0,0]

    sourceId = vtk.vtkIdList()
    sourceId.SetNumberOfIds(1)

    sourcePosition = self.points.GetPoint(0)

    source = inputPolyData.FindPoint(sourcePosition)

    sourceId.InsertId(0,source)

    targetPosition = [0,0,0]
    targetId = vtk.vtkIdList()
    targetId.SetNumberOfIds(STpoints.GetNumberOfFiducials()-1) 
      
    for i in range(1, STpoints.GetNumberOfFiducials()):
      STpoints.GetNthFiducialPosition(i,targetPosition)
 
      target = inputPolyData.FindPoint(targetPosition)
      targetId.InsertId(i-1,target)

    pathCreation = vtkSlicerVMTKFunctionalitiesModuleLogic.vtkvmtkPolyDataCenterlines()

    # Multiple paths for different ROIs are created!

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
    
      self.createdPath = pathCreation.GetOutput()

      self.pathSmoothing(self.createdPath)

      ############################ Create The 3D Model Of The Path And Add It To The Scene ############################################# 

      model = slicer.vtkMRMLModelNode()
      model.SetScene(slicer.mrmlScene)
      model.SetName(slicer.mrmlScene.GenerateUniqueName("PathModel"))
      model.SetAndObservePolyData(self.createdPath)

      # Create display node
      modelDisplay = slicer.vtkMRMLModelDisplayNode()
      modelDisplay.SetColor(1,1,0) # yellow
      modelDisplay.SetScene(slicer.mrmlScene)
      slicer.mrmlScene.AddNode(modelDisplay)
      model.SetAndObserveDisplayNodeID(modelDisplay.GetID())

      # Add to scene
      if vtk.VTK_MAJOR_VERSION <= 5:
        # shall not be needed.
        modelDisplay.SetInputPolyData(model.GetPolyData())
      slicer.mrmlScene.AddNode(model)

      self.pathModelNamesList.append(model.GetName()) # Save names of the models to delete them before creating the new ones.

      ########################################### Merge Centerline Points with Path Points ###############################################
      if self.points.GetNumberOfPoints()>0:
        pathPoints = self.createdPath.GetPoints()
        for j in xrange(pathPoints.GetNumberOfPoints()):
          self.points.InsertNextPoint(pathPoints.GetPoint(j))

      markupLogic = slicer.modules.markups.logic()
      markupLogic.SetActiveListID(STpoints)
      #self.createdPath = None

    return True
     
  def pathSmoothing(self, pathModel):
      
    import vtkSlicerVMTKFunctionalitiesModuleLogic
    
    NumberOfPoints = pathModel.GetNumberOfPoints()
    position = NumberOfPoints-1
    startingPoint = [0,0,0]
    pathModel.GetPoint(position,startingPoint)
    #print "Starting point centerline: ",startingPoint
    targetPosition = [0,0,0]
    pathModel.GetPoint(1,targetPosition)
           
    squaredDist = vtk.vtkMath.Distance2BetweenPoints(startingPoint,targetPosition)
    #print squaredDist 
    
    if (squaredDist < 10.000) :
      smoothfactor = 1
      iterations = 1000
    else:
      smoothfactor = 1
      iterations = 100
      
    centerlineSmoothing = vtkSlicerVMTKFunctionalitiesModuleLogic.vtkvmtkCenterlineSmoothing()
    centerlineSmoothing.SetInputData(pathModel)
    centerlineSmoothing.SetNumberOfSmoothingIterations(iterations)
    centerlineSmoothing.SetSmoothingFactor(smoothfactor)
    centerlineSmoothing.Update()
    
    self.createdPath = centerlineSmoothing.GetOutput()

  def CreateFiducialsPath(self, pathPolyData, fiducialList):
    NoP = pathPolyData.GetNumberOfPoints()    
    NthFiducial = 0
    point = [0,0,0]
    for i in xrange(NoP):
      pathPolyData.GetPoint(i,point)
      fiducialList.AddFiducial(point[0],point[1],point[2])
      NthFiducial += 1

    displayNode = fiducialList.GetDisplayNode()
    displayNode.SetVisibility(0)

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
      self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,156,126)")

      if self.fiducialListSelector.currentNode():
        self.fiducialNode = self.fiducialListSelector.currentNode()
        displayNode = self.fiducialNode.GetDisplayNode()
        displayNode.SetVisibility(0)

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

      self.onResetCameraButtonPressed()

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

      needleModelNodes = slicer.mrmlScene.GetNodesByName('ProbeModel')
      if needleModelNodes.GetNumberOfItems() > 0:
        probeNode = needleModelNodes.GetItemAsObject(0)
        probeDisplayNode = probeNode.GetDisplayNode()
        probeDisplayNode.SetColor(0.4, 1.0, 0.0)
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

      ################## Camera is connected to the transform if needed #####################

      self.cameraForNavigation.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
 
      lm = slicer.app.layoutManager()
      yellowWidget = lm.sliceWidget('Yellow')
      self.yellowLogic = yellowWidget.sliceLogic()
      redWidget = lm.sliceWidget('Red')
      self.redLogic = redWidget.sliceLogic() 
      greenWidget = lm.sliceWidget('Green')
      self.greenLogic = greenWidget.sliceLogic()

      fiducialPos = [0,0,0]
      if self.points.GetNumberOfPoints()>0:
        for i in xrange(self.points.GetNumberOfPoints()):
          point = self.points.GetPoint(i)
          #print point
          p = [point[0],point[1],point[2]]
          self.pointsList.append(p)

      '''if self.pathCreated == 1:
        pathFiducialsCollection = slicer.mrmlScene.GetNodesByName('pathFiducials')
        for j in xrange(pathFiducialsCollection.GetNumberOfItems()):
          fidNode = pathFiducialsCollection.GetItemAsObject(j)
          for n in xrange(fidNode.GetNumberOfFiducials()):
            fidNode.GetNthFiducialPosition(n,fiducialPos)
            s = [fiducialPos[0],fiducialPos[1],fiducialPos[2]]
            self.pointsList.append(s)''' 

      camera = self.cameraForNavigation.GetCamera()
      camera.SetClippingRange(0.7081381565016212, 708.1381565016211) # to be checked
      self.sensorTimer.start()
       
    else:  # When button is released...
      self.ProbeTrackButton.setStyleSheet("background-color: rgb(255,255,255)")
      self.ResetCameraButton.enabled = False

      self.sensorTimer.stop()
      self.cNode.Stop()
      #self.cNode = None
      #self.cameraForNavigation = None
      #self.probeCalibrationTransform = None

      lastFPBeforeStoppingTracking = [0,0,0]
      lastPosBeforeStoppingTracking = [0,0,0]

      self.cameraForNavigation.GetFocalPoint(lastFPBeforeStoppingTracking)
      self.cameraForNavigation.GetPosition(lastPosBeforeStoppingTracking)

      self.enableSelectors()
      self.onSelect()

      self.cameraForNavigation.SetFocalPoint(lastFPBeforeStoppingTracking[0],lastFPBeforeStoppingTracking[1],lastFPBeforeStoppingTracking[2])
      self.cameraForNavigation.SetPosition(lastPosBeforeStoppingTracking[0],lastPosBeforeStoppingTracking[1],lastPosBeforeStoppingTracking[2])
      camera = self.cameraForNavigation.GetCamera()
      camera.SetClippingRange(0.7081381565016212, 708.1381565016211) # to be checked

      if self.points.GetNumberOfPoints() > 0:
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
    cameraNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLCameraNode')
    self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
    if cameraNodes.GetNumberOfItems() > 0:
      if self.cameraForNavigation.GetTransformNodeID() == None:
        self.cameraForNavigation.SetPosition(-1.0,0.0,0.0)
        self.cameraForNavigation.SetFocalPoint(-5.0,0.0,0.0)
      else:
        tNodeCollections = slicer.mrmlScene.GetNodesByName('centerlineCompensationTransform')
        if tNodeCollections.GetNumberOfItems() > 0:
          tNode = tNodeCollections.GetItemAsObject(0)
          transformMatrix = vtk.vtkMatrix4x4()
          tNode.GetMatrixTransformToParent(transformMatrix)
          pos = [0,0,0]
          pos[0] = transformMatrix.GetElement(0,3)
          pos[1] = transformMatrix.GetElement(1,3)
          pos[2] = transformMatrix.GetElement(2,3)
          self.cameraForNavigation.SetPosition(pos[0],pos[1],pos[2])
          fp = [0,0,0]
          self.cameraForNavigation.GetFocalPoint(fp)
          self.cameraForNavigation.SetFocalPoint(fp[0],fp[1],fp[2])
      
  def CheckCurrentPosition(self, tMatrix):

    distance = []

    '''if self.fiducialNode == None:
      fiducialNodesCollection = slicer.mrmlScene.GetNodesByName('CenterlineFiducials')
      self.fiducialNode = fiducialNodesCollection.GetItemAsObject(0)'''

    originalCoord = [0,0,0]
    originalCoord[0] = tMatrix.GetElement(0,3)
    originalCoord[1] = tMatrix.GetElement(1,3)
    originalCoord[2] = tMatrix.GetElement(2,3)

    originalCoord = numpy.asarray(originalCoord)

    distance = ((self.pointsList-originalCoord)**2).sum(axis=1)
    ndx = distance.argsort()
    closestPoint = self.pointsList[ndx[0]]

    tMatrix.SetElement(0,3,closestPoint[0])
    tMatrix.SetElement(1,3,closestPoint[1])
    tMatrix.SetElement(2,3,closestPoint[2])

    ####################################################################################################################
    # Continuosly Update ViewUp Of The Camera To Always Have It On One Direction Orthogonal To The Locator's Long Axis #
    ####################################################################################################################

    x = closestPoint[0]
    y = closestPoint[1]
    z = closestPoint[2]          
    c = (x+y)/z
    viewUp = [-1,-1,c]
    self.cameraForNavigation.SetViewUp(viewUp)

    self.yellowLogic.SetSliceOffset(x)
    self.greenLogic.SetSliceOffset(y)
    self.redLogic.SetSliceOffset(z)

    self.centerlineCompensationTransform.SetMatrixTransformToParent(tMatrix)
