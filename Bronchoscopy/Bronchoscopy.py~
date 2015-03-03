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
    This file was originally developed by Pietro Nardelli and Alberto Corvo, University College Cork.
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
    # Flythrough Variables    
    #
    self.transform = None
    self.path = None
    self.camera = None
    self.skip = 0
    self.fiducialNode = None

    self.timer = qt.QTimer()
    self.timer.setInterval(20)
    self.timer.connect('timeout()', self.flyToNext)

    #
    # Sensor Tracking Variables
    #
    self.sensorTimer = qt.QTimer()
    self.sensorTimer.setInterval(1)
    self.sensorTimer.connect('timeout()', self.ReadPosition)

    self.track = slicer.vtkMRMLModelNode()
    self.ModelDisplay = slicer.vtkMRMLModelDisplayNode()

    self.coordinates = []

    self.needleCalibrationTransform = None
    self.cameraForNavigation = None
    self.cNode = None

    self.updateGUI()

    if not parent:
      self.setup()
      self.virtualCameraNodeSelector.setMRMLScene(slicer.mrmlScene)
      self.parent.show()
      self.updateGUI()

  def setup(self):
    #
    # Reload and Test area
    #
    reloadCollapsibleButton = ctk.ctkCollapsibleButton()
    reloadCollapsibleButton.text = "Reload && Test"
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

    #
    # Parameters Area
    #
    parametersCollapsibleButton = ctk.ctkCollapsibleButton()
    parametersCollapsibleButton.text = "Bronchoscopy Path Creation"
    self.layout.addWidget(parametersCollapsibleButton)
    self.layout.setSpacing(20)
    # Layout within the dummy collapsible button
    IOFormLayout = qt.QFormLayout(parametersCollapsibleButton)

    #
    # 3D Model Selector
    #
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
      modelDisplayNode.SetFrontfaceCulling(0)
      modelDisplayNode.SetBackfaceCulling(1)

    #
    # Label Selector
    #
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

    #
    # Fiducial Selector
    #

    self.fiducialsListSelector = slicer.qMRMLNodeComboBox()
    self.fiducialsListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.fiducialsListSelector.selectNodeUponCreation = False
    self.fiducialsListSelector.addEnabled = True
    self.fiducialsListSelector.removeEnabled = True
    self.fiducialsListSelector.noneEnabled = False
    self.fiducialsListSelector.showHidden = False
    self.fiducialsListSelector.showChildNodeTypes = False
    self.fiducialsListSelector.setMRMLScene( slicer.mrmlScene )
    self.fiducialsListSelector.setToolTip( "Place a source and a target point within the 3D model." )
    IOFormLayout.addRow("Source and Target seeds: ", self.fiducialsListSelector)
    
    #
    # Create Path Button
    #
    self.CreatePathButton = qt.QPushButton("Create Path")
    self.CreatePathButton.toolTip = "Run the algorithm to create path between fiducials."
    self.CreatePathButton.setFixedSize(200,50)
    if( self.inputSelector.currentNode() and self.fiducialsListSelector.currentNode()):
        self.CreatePathButton.enabled = True
    else:
        self.CreatePathButton.enabled = False
    IOFormLayout.addWidget(self.CreatePathButton)

    #
    # Virtual Navigation Flythrough Collapsible Button
    #
    self.flythroughCollapsibleButton = ctk.ctkCollapsibleButton()
    self.flythroughCollapsibleButton.text = "Virtual Navigation"
    self.flythroughCollapsibleButton.setChecked(False)
    #self.flythroughCollapsibleButton.setFixedSize(400,40)
    self.flythroughCollapsibleButton.enabled = False
    self.layout.addWidget(self.flythroughCollapsibleButton)
    flythroughFormLayout = qt.QFormLayout(self.flythroughCollapsibleButton)
    
    #
    # Virtual Navigation Camera Node Selector
    #
    self.virtualCameraNodeSelector = slicer.qMRMLNodeComboBox()
    self.virtualCameraNodeSelector.objectName = 'virtualCameraNodeSelector'
    self.virtualCameraNodeSelector.toolTip = "Select a camera that will fly along the path for the virtual navigation."
    self.virtualCameraNodeSelector.nodeTypes = ['vtkMRMLCameraNode']
    self.virtualCameraNodeSelector.noneEnabled = False
    self.virtualCameraNodeSelector.addEnabled = False
    self.virtualCameraNodeSelector.removeEnabled = False
    self.virtualCameraNodeSelector.enabled = False
    self.virtualCameraNodeSelector.showHidden = True
    flythroughFormLayout.addRow("Camera:", self.virtualCameraNodeSelector)
    self.parent.connect('mrmlSceneChanged(vtkMRMLScene*)', 
                        self.virtualCameraNodeSelector, 'setMRMLScene(vtkMRMLScene*)')
    #
    # Frame Slider
    #
    self.frameSlider = ctk.ctkSliderWidget()
    self.frameSlider.connect('valueChanged(double)', self.frameSliderValueChanged)
    self.frameSlider.decimals = 0
    flythroughFormLayout.addRow("Frame:", self.frameSlider)
    
    #
    # Frame Skip Slider
    #
    self.frameSkipSlider = ctk.ctkSliderWidget()
    self.frameSkipSlider.connect('valueChanged(double)', self.frameSkipSliderValueChanged)
    self.frameSkipSlider.decimals = 0
    self.frameSkipSlider.minimum = 0
    self.frameSkipSlider.maximum = 10
    flythroughFormLayout.addRow("Frame skip:", self.frameSkipSlider)
    
    #
    # Frame Delay Slider
    #
    self.frameDelaySlider = ctk.ctkSliderWidget()
    self.frameDelaySlider.connect('valueChanged(double)', self.frameDelaySliderValueChanged)
    self.frameDelaySlider.decimals = 0
    self.frameDelaySlider.minimum = 0
    self.frameDelaySlider.maximum = 500
    self.frameDelaySlider.suffix = " ms"
    self.frameDelaySlider.value = 50
    flythroughFormLayout.addRow("Frame delay:", self.frameDelaySlider)
    
    #
    # View Angle Slider
    #
    self.viewAngleSlider = ctk.ctkSliderWidget()
    self.viewAngleSlider.connect('valueChanged(double)', self.viewAngleSliderValueChanged)
    self.viewAngleSlider.decimals = 0
    self.viewAngleSlider.minimum = 30
    self.viewAngleSlider.maximum = 180
    flythroughFormLayout.addRow("View Angle:", self.viewAngleSlider)
    
    #
    # Create Virtual Navigation Button
    #
    self.NavigationButton = qt.QPushButton("Play")
    self.NavigationButton.toolTip = "Fly through path."
    self.NavigationButton.checkable = True
    self.NavigationButton.enabled = False
    self.NavigationButton.setFixedSize(200,50)
    flythroughFormLayout.addWidget(self.NavigationButton)

    #
    # 'Track Sensor' Collapsible Button 
    #
    trackerCollapsibleButton = ctk.ctkCollapsibleButton()
    trackerCollapsibleButton.text = "Sensor Tracking"
    self.layout.addWidget(trackerCollapsibleButton)
    self.layout.setSpacing(20)
    trackerFormLayout = qt.QFormLayout(trackerCollapsibleButton)

    #
    # Text Output Label for Sensor Tracking Position 
    #
    self.RealPosition = qt.QLineEdit()
    self.RealPosition.setReadOnly(False)
    self.RealPosition.textChanged.connect(self.RealPositionValueChanged)
    trackerFormLayout.addRow("Sensor Position", self.RealPosition)

    #
    # Matlab Track Button
    #
    self.MatlabTrackButton = qt.QPushButton("Track Sensor")
    self.MatlabTrackButton.toolTip = "Take Sensor output."
    self.MatlabTrackButton.setFixedSize(300,40)
    self.MatlabTrackButton.checkable = True
    if( self.inputSelector.currentNode() ):
        self.MatlabTrackButton.enabled = True
    else:
        self.MatlabTrackButton.enabled = False
    
    trackerFormLayout.addWidget(self.MatlabTrackButton)

    #
    # Create Connections
    #
    self.CreatePathButton.connect('clicked(bool)', self.onCreatePathButton)
    self.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.labelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.fiducialsListSelector.connect("currentNodeChanged(vtkMRMLNode*)",self.onSelect)
    self.NavigationButton.connect('toggled(bool)', self.onNavigationButtonToggled)
    self.virtualCameraNodeSelector.connect('currentNodeChanged(vtkMRMLNode*)', self.setCameraNode)
    self.MatlabTrackButton.connect('toggled(bool)', self.onMatlabTrackButtonToggled)
    
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
    if( self.inputSelector.currentNode() and self.labelSelector.currentNode() and self.fiducialsListSelector.currentNode() ):
      self.CreatePathButton.enabled = True
    else:
      self.CreatePathButton.enabled = False
      self.NavigationButton.enabled = False

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


################################### CREATE PATH ######################################## 

  def onCreatePathButton(self):
    print("Create the paths between fiducials")
    self.CreatePathButton.enabled = False
    self.flythroughCollapsibleButton.enabled = False
    self.NavigationButton.enabled = False
    self.MatlabTrackButton.enabled = False
    self.createPath(self.labelSelector.currentNode()) #, self.fiducialsListSelector.currentNode())
    self.CreatePathButton.enabled = True
    self.MatlabTrackButton.enabled = True
    self.updateGUI()

  def createPath(self,labelVolume): #,fiducialList):
    """
    Run the actual algorithm to create the path between the 2 fiducials
    """
    """import vtkSlicerCenterlineExtractionModuleLogic

    #if( fiducialList.GetNumberOfMarkups() > 2 ):
        #return False
    
    modelNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLModelNode')
    for n in xrange( modelNodes.GetNumberOfItems() ):
      node = modelNodes.GetItemAsObject(n)
      nodeName = node.GetName()
      replaceName = nodeName.replace('-', ' - ')
      splitName = replaceName.split()
      for item in splitName:
        if (item == 'Path') or (item == 'Cursor') or (item == 'Transform'):
          slicer.mrmlScene.RemoveNode(node)
                 
    inputPolyData = inputVolume.GetPolyData()
    self.delayDisplay('Computing the path')
    self.Picking(inputPolyData, fiducialList)
        
    centerlineExtraction = vtkSlicerCenterlineExtractionModuleLogic.vtkvmtkPolyDataCenterlines()
    
    for t in xrange(self.Target.GetNumberOfIds()):
   
      target = vtk.vtkIdList()
      target.SetNumberOfIds(1) 
      target.InsertId(0,self.Target.GetId(t))

      centerlineExtraction.SetInputData(inputPolyData)
      centerlineExtraction.SetSourceSeedIds(self.Source)
      centerlineExtraction.SetTargetSeedIds(target)
      centerlineExtraction.SetRadiusArrayName('MaximumInscribedSphereRadius')
      centerlineExtraction.SimplifyVoronoiOn();
      centerlineExtraction.CenterlineResamplingOn()
      centerlineExtraction.SetCostFunction('1/R')
      centerlineExtraction.GenerateDelaunayTessellationOn()
      centerlineExtraction.Update()
    
      self.centerline = centerlineExtraction.GetOutput()



      #
      # Smooth the path to get a better result
      #
      #self.Smoothing(self.centerline)

      nodeType = 'vtkMRMLMarkupsFiducialNode'
      self.fiducialNode = slicer.mrmlScene.CreateNodeByClass(nodeType)
      self.fiducialNode.SetScene(slicer.mrmlScene)
      self.fiducialNode.SetName('CenterlineFiducial')
      slicer.mrmlScene.AddNode(self.fiducialNode)
    
      self.CreateFiducialPath(self.centerline, self.fiducialNode)
      result = BronchoscopyComputePath(self.fiducialNode)
      print "-> Computed path contains %d elements" % len(result.path)
      model = BronchoscopyPathModel(result.path, self.fiducialNode)
      print "-> Model created"

      slicer.mrmlScene.RemoveNode(self.fiducialNode)
      markupNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLMarkupsFiducialNode')
      numberOfMarkupNode = markupNodes.GetNumberOfItems()
      lastMarkupNode = markupNodes.GetItemAsObject(numberOfMarkupNode-1)
      markupLogic = slicer.modules.markups.logic()
      markupLogic.SetActiveListID(lastMarkupNode)"""

    self.centerline = slicer.vtkMRMLScalarVolumeNode()
    #self.centerline.addAttribute( "vtkMRMLScalarVolumeNode", "LabelMap", 1 )
    slicer.mrmlScene.AddNode( self.centerline )

    centerlineExtraction = slicer.modules.thinning
    parameters = {
        "inputVolume": labelVolume.GetID(),
        "outputVolume": self.centerline.GetID(),	  
        }
    slicer.cli.run( centerlineExtraction,None,parameters,wait_for_completion=True )

    # create 3D model of the centerline
    nodeType = 'vtkMRMLModelHierarchyNode'
    modelHierarchy = slicer.mrmlScene.CreateNodeByClass(nodeType)
    modelHierarchy.SetScene(slicer.mrmlScene)
    modelHierarchy.SetName(slicer.mrmlScene.GetUniqueNameByString('CenterlineModelHierarchy'))
    slicer.mrmlScene.AddNode(modelHierarchy)

    parameters = {}
    parameters["InputVolume"] = self.centerline.GetID()
    parameters["ModelSceneFile"] = modelHierarchy.GetID()
    parameters["Name"] = 'CenterlineModel'
    #parameters["FilterType"] = 'Laplacian'
    parameters["Smooth"] = 0
    parameters["Decimate"] = 0.00
    
    modelMaker = slicer.modules.modelmaker
    slicer.cli.run(modelMaker, None, parameters,True)

    # create fiducial path
    nodeType = 'vtkMRMLMarkupsFiducialNode'
    self.fiducialNode = slicer.mrmlScene.CreateNodeByClass(nodeType)
    self.fiducialNode.SetScene(slicer.mrmlScene)
    self.fiducialNode.SetName('CenterlineFiducial')
    slicer.mrmlScene.AddNode(self.fiducialNode)

    modelsCollection = slicer.mrmlScene.GetNodesByClass('vtkMRMLModelNode')
    numberOfItems = modelsCollection.GetNumberOfItems()
    self.centerlineModel = modelsCollection.GetItemAsObject(numberOfItems-1)

    self.polydata = self.centerlineModel.GetPolyData()
    self.points = vtk.vtkPoints()

    iterations = 10
    self.Smoothing(self.polydata, iterations)
    self.CreateFiducialPath(self.points, self.fiducialNode)

    # Create model node
    #model = slicer.vtkMRMLModelNode()
    #model.SetScene(slicer.mrmlScene)
    #model.SetName(slicer.mrmlScene.GenerateUniqueName("Path-%s" % self.fiducialNode.GetName()))
    #model.SetAndObservePolyData(self.polydata)

    # Create display node
    #modelDisplay = slicer.vtkMRMLModelDisplayNode()
    #modelDisplay.SetColor(1,0,0) # yellow
    #modelDisplay.SetScene(slicer.mrmlScene)
    #slicer.mrmlScene.AddNode(modelDisplay)
    #model.SetAndObserveDisplayNodeID(modelDisplay.GetID())

    # Add to scene
    #if vtk.VTK_MAJOR_VERSION <= 5:
      # shall not be needed.
    #  modelDisplay.SetInputPolyData(model.GetPolyData())
    #slicer.mrmlScene.AddNode(model)

    #self.CreateFiducialPath(self.centerlineModel, self.fiducialNode)
   
    #result = BronchoscopyComputePath(self.fiducialNode)
    #print "-> Computed path contains %d elements" % len(result.path)
    #model = BronchoscopyPathModel(self.fiducialNode) #(result.path, self.fiducialNode)
    #print "-> Model created"

    #
    # Update Frame Slider Range
    #
    #if( fiducialList.GetNumberOfMarkups() == 2 ):
     # self.frameSlider.maximum = len(result.path) - 2
    self.frameSlider.maximum = self.fiducialNode.GetNumberOfFiducials() - 2
      #
      # Update Flythrough Variables
      #
    self.camera = self.camera

    # Camera cursor
    sphere = vtk.vtkSphereSource()
    sphere.Update()
     
    # Create model node
    cursor = slicer.vtkMRMLModelNode()
    cursor.SetScene(slicer.mrmlScene)
    cursor.SetName(slicer.mrmlScene.GenerateUniqueName("Cursor-%s" % self.fiducialNode.GetName()))
    if vtk.VTK_MAJOR_VERSION <= 5:
      cursor.SetAndObservePolyData(sphere.GetOutput())
    else:
      cursor.SetPolyDataConnection(sphere.GetOutputPort())

    # Create display node
    cursorModelDisplay = slicer.vtkMRMLModelDisplayNode()
    cursorModelDisplay.SetColor(1,0,0) # red
    cursorModelDisplay.SetScene(slicer.mrmlScene)
    slicer.mrmlScene.AddNode(cursorModelDisplay)
    cursor.SetAndObserveDisplayNodeID(cursorModelDisplay.GetID())

    # Add to scene
    if vtk.VTK_MAJOR_VERSION <= 5:
      # Shall not be needed.
      cursorModelDisplay.SetInputPolyData(sphere.GetOutput())
    slicer.mrmlScene.AddNode(cursor)

    # Create transform node
    transform = slicer.vtkMRMLLinearTransformNode()
    transform.SetName(slicer.mrmlScene.GenerateUniqueName("Transform-%s" % self.fiducialNode.GetName()))
    slicer.mrmlScene.AddNode(transform)
    cursor.SetAndObserveTransformNodeID(transform.GetID())
    
    self.transform = transform
    toParent = vtk.vtkMatrix4x4()
    self.transform.GetMatrixTransformToParent(toParent)
    index = [0,0,0]
    self.fiducialNode.GetNthFiducialPosition(0,index)
    toParent.SetElement(0 ,3, index[0])
    toParent.SetElement(1, 3, index[1])
    toParent.SetElement(2, 3, index[2])
    self.transform.SetMatrixTransformToParent(toParent)


    #self.transform = model.transform
    #self.path = result.path
    
      #
      # Enable/Disable Flythrough Button
      #
    self.flythroughCollapsibleButton.enabled = True #len(result.path) > 0
    self.NavigationButton.enabled = True #len(result.path) > 0
    self.MatlabTrackButton.enabled = True

    return True

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
  
  def CreateFiducialPath(self, centerlineModelNode, fiducialList):

    NoP = centerlineModelNode.GetNumberOfPoints()
    
    NthFiducial = 0
    point = [0,0,0]
    prova = [0,0,0]
    for i in xrange(NoP):
      centerlineModelNode.GetPoint(i,point)
      fiducialList.AddFiducial(point[0],point[1],point[2])
      fiducialList.SetNthMarkupVisibility(NthFiducial,0)
      NthFiducial += 1

  def Smoothing(self, pathModel, iterationsNumber):
    
    NumberOfCells = pathModel.GetNumberOfCells()

    for i in range(NumberOfCells-50,8,-8):
      cell0 = pathModel.GetCell(i+8)

      points0 = cell0.GetPoints()

      if( points0.GetNumberOfPoints() % 2 == 0 ):
        pos0 = points0.GetNumberOfPoints()/2
      else:
   	pos0 = int(points0.GetNumberOfPoints())/2
      point0 = [0,0,0]        
      points0.GetPoint(pos0,point0)

      cell1 = pathModel.GetCell(i)
      points1 = cell1.GetPoints()

      if( points1.GetNumberOfPoints() % 2 == 0 ):
        pos1 = points1.GetNumberOfPoints()/2
      else:
   	pos1 = int(points1.GetNumberOfPoints())/2

      point1 = [0,0,0]
      points1.GetPoint(pos1,point1)

      cell2 = pathModel.GetCell(i-8)
      points2 = cell2.GetPoints()

      if( points2.GetNumberOfPoints() % 2 == 0 ):
        pos2 = points2.GetNumberOfPoints()/2
      else:
   	pos2 = int(points2.GetNumberOfPoints())/2

      point2 = [0,0,0] 
      points2.GetPoint(pos2,point2)

      relaxation = 0.5

      if( i > NumberOfCells - 50 ):
        if( abs(point1[0]-point0[0])<=3 and abs(point1[0]-point2[0])<=3):
          point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
          point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
          point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);
      else:
        if( abs(point1[0]-point0[0])<=3 and abs(point1[0]-point2[0])<=3 and
            abs(point1[1]-point0[1])<=2 and abs(point1[1]-point2[1])<=2 ):
          point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
          point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
          point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);

      self.points.InsertNextPoint(*point1)
  
    if( iterationsNumber > 1 ):
      point0 = [0,0,0]
      point1 = [0,0,0]
      point2 = [0,0,0]
      for iterations in xrange(1,iterationsNumber):
        for z in xrange(1,self.points.GetNumberOfPoints()-1,1):
          self.points.GetPoint(z-1,point0)
          self.points.GetPoint(z,point1)
          self.points.GetPoint(z+1,point2)
 
          if( z < 100 ): #to be modified
            if( abs(point1[0]-point0[0])<=3 and abs(point1[0]-point2[0])<=3):
              point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
              point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
              point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);
          else:   
            if( abs(point1[0]-point0[0])<=3 and abs(point1[0]-point2[0])<=3 and 
                abs(point1[1]-point0[1])<=2 and abs(point1[1]-point2[1])<=2 ):
              point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
              point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
              point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);

            elif(abs(point1[0]-point0[0])>3 or abs(point1[0]-point2[0])>3 and 
                 abs(point1[1]-point0[1])>2 or abs(point1[1]-point2[1])>2 ):
	      FoundPrev = 0
              FoundNext = 0
	      newPoint0 = [0,0,0]
	      newPoint2 = [0,0,0]       
              if(z > 15):
                n = z-1
                prev = 1
	        while( prev < 15 and not(FoundPrev) ):
                  m = n-prev
                  self.points.GetPoint(m,point0)
		  prev = prev+1
		  if( abs(point1[0]-point0[0])<=3 and 
                      abs(point1[1]-point0[1])<=2 and 
                      abs(point1[2]-point0[2])<=100 ):
		    FoundPrev = 1
		    newPoint0[0] = point0[0]
		    newPoint0[1] = point0[1]
		    newPoint0[2] = point0[2]
	            
                if(self.points.GetNumberOfPoints()-z > 15):
                  k = z+1
                  next = 1
	          while(next < 15 and not(FoundNext)):
                    j = k+next
                    self.points.GetPoint(j,point2)
		    next = next+1
                    if( abs(point1[0]-point2[0])<=3 and 
                        abs(point1[1]-point2[1])<=2 and
                        abs(point1[2]-point2[2])<=100 ):
		      FoundNext = 1
		      newPoint2[0] = point2[0]
		      newPoint2[1] = point2[1]
		      newPoint2[2] = point2[2]

	
              if(FoundPrev and FoundNext):
	        point1[0] += relaxation * (0.5 * (newPoint0[0] + newPoint2[0]) - point1[0]);
                point1[1] += relaxation * (0.5 * (newPoint0[1] + newPoint2[1]) - point1[1]);
                point1[2] += relaxation * (0.5 * (newPoint0[2] + newPoint2[2]) - point1[2]);
		    
            elif(abs(point1[0]-point0[0]) > 3 or abs(point1[1]-point0[1]) > 2):
              FoundPrev = 0
	      n = z-1
              if(z > 15):
                prev = 1
	        while(prev < 15 and not(FoundPrev)):
                  m = n-prev
                  self.points.GetPoint(m,point0)
	          prev = prev+1
                  if(abs(point1[0]-point0[0])<=3 and abs(point1[1]-point0[1])<=2):
		    FoundPrev = 1
	      
              if(FoundPrev):
	        point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
                point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
                point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);  
			
            elif(abs(point1[0]-point2[0]) > 3 or abs(point1[1]-point2[1]) > 2):
              FoundNext = 0
	      n = z+1
              if(self.points.GetNumberOfPoints()-z > 15):
                next = 1
	        while(next < 15 and not(FoundNext)):
                  m = n+next
                  self.points.GetPoint(m,point2)
		  next = next+1
                  if(abs(point1[0]-point2[0])<=3 and abs(point1[1]-point2[1])<=2):
		    FoundNext = 1
              
              if(FoundNext):
	        point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
                point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
                point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);

          self.points.SetPoint(z, *point1)

  #for (int i=0; i<numberOfIterations; i++)
  #  {
  #  for (int j=1; j<numberOfPoints-1; j++)
  #    {
  #    smoothLinePoints->GetPoint(j-1,point0);
  #    smoothLinePoints->GetPoint(j  ,point1);
  #    smoothLinePoints->GetPoint(j+1,point2);

   #   point1[0] += relaxation * (0.5 * (point0[0] + point2[0]) - point1[0]);
   #   point1[1] += relaxation * (0.5 * (point0[1] + point2[1]) - point1[1]);
   #   point1[2] += relaxation * (0.5 * (point0[2] + point2[2]) - point1[2]);

    #  smoothLinePoints->SetPoint(j,point1);
    #  }
   # }


    #import vtkSlicerCenterlineExtractionModuleLogic

    #NumberOfPoints = pathModel.GetNumberOfPoints()
    #print NumberOfPoints
    #position = NumberOfPoints-1
    #startingPoint = [0,0,0]
    #pathModel.GetPoint(position,startingPoint)
    #print "Starting point centerline: ",startingPoint
    #targetPosition=[0,0,0]
    #pathModel.GetPoint(1,targetPosition)
           
    #squaredDist = vtk.vtkMath.Distance2BetweenPoints(startingPoint,targetPosition)
    #print squaredDist 
    
    #if (squaredDist < 10.000) :
    #smoothfactor=1
    #iterations=100
      #print iterations
   # else:
     # smoothfactor=1
     # iterations=10
      #print iterations

    #centerlineSmoothing = vtkSlicerCenterlineExtractionModuleLogic.vtkvmtkCenterlineSmoothing()
    #centerlineSmoothing.SetInputData(pathModel)
    #centerlineSmoothing.SetNumberOfSmoothingIterations(iterations)
    #centerlineSmoothing.SetSmoothingFactor(smoothfactor)
    #centerlineSmoothing.Update()
    
    #self.polydata = centerlineSmoothing.GetOutput()

############################# VIRTUAL NAVIGATION #################################

  def onNavigationButtonToggled(self, checked):
    if checked:
      self.CreatePathButton.enabled = False
      self.MatlabTrackButton.enabled = False
      self.timer.start()
      self.NavigationButton.text = "Stop Navigation"
    else:
      self.CreatePathButton.enabled = True
      self.MatlabTrackButton.enabled = True
      self.timer.stop()
      self.NavigationButton.text = "Play Navigation"

  def setCameraNode(self, newCameraNode):
    """Allow to set the current camera node. 
    Connected to signal 'currentNodeChanged()' emitted by camera node selector."""
    
    #
    # Remove Previous Observer
    #
    if self.cameraNode and self.cameraNodeObserverTag:
      self.cameraNode.RemoveObserver(self.cameraNodeObserverTag)
    if self.camera and self.cameraObserverTag:
      self.camera.RemoveObserver(self.cameraObserverTag)
    
    newCamera = None
    if newCameraNode:
      newCamera = newCameraNode.GetCamera()
      # Add CameraNode ModifiedEvent observer
      self.cameraNodeObserverTag = newCameraNode.AddObserver(vtk.vtkCommand.ModifiedEvent, self.onCameraNodeModified)
      # Add Camera ModifiedEvent observer
      self.cameraObserverTag = newCamera.AddObserver(vtk.vtkCommand.ModifiedEvent, self.onCameraNodeModified)
      
    self.cameraNode = newCameraNode
    self.camera = newCamera
    
    #
    # Update UI
    #    
    self.updateWidgetFromMRML()
  
  def updateWidgetFromMRML(self):
    if self.camera:
        self.viewAngleSlider.value = self.camera.GetViewAngle()
    if self.cameraNode:
        pass
    
  def onCameraModified(self, observer, eventid):
    self.updateWidgetFromMRML()
    
  def onCameraNodeModified(self, observer, eventid):
    self.updateWidgetFromMRML()    
   
  def frameSliderValueChanged(self, newValue):
    #print "frameSliderValueChanged:", newValue
    self.flyTo(newValue)
    
  def frameSkipSliderValueChanged(self, newValue):
    #print "frameSkipSliderValueChanged:", newValue
    self.skip = int(newValue)
    
  def frameDelaySliderValueChanged(self, newValue):
    #print "frameDelaySliderValueChanged:", newValue
    self.timer.interval = newValue
    
  def viewAngleSliderValueChanged(self, newValue):
    if not self.cameraNode:
      return
    #print "viewAngleSliderValueChanged:", newValue
    self.cameraNode.GetCamera().SetViewAngle(newValue)
      
  def flyToNext(self):
    currentStep = self.frameSlider.value
    nextStep = currentStep + self.skip + 1
    if nextStep > self.fiducialNode.GetNumberOfFiducials() - 2: #len(self.path) - 2:
      nextStep = 0
    self.frameSlider.value = nextStep
    
  def flyTo(self, f):
    """ Apply the fth step in the path to the global camera"""
    #if self.path:
    f = int(f)
      #p = self.path[f]
    p = [0,0,0]
    foc = [0,0,0]
    if self.fiducialNode:
      self.fiducialNode.GetNthFiducialPosition(f,p)
      self.camera.SetPosition(p)
      #foc = self.path[f+1]
      self.fiducialNode.GetNthFiducialPosition(f+1,foc)
      self.camera.SetFocalPoint(foc)

      toParent = vtk.vtkMatrix4x4()
      self.transform.GetMatrixTransformToParent(toParent)
      toParent.SetElement(0, 3, p[0])
      toParent.SetElement(1, 3, p[1])
      toParent.SetElement(2, 3, p[2])
      self.transform.SetMatrixTransformToParent(toParent)

############################# SENSOR TRACKING #################################

  def onMatlabTrackButtonToggled(self, checked):     
    if checked:
      #self.carina = 0
      #self.fiducialNumber = 0
      #self.CreatePathButton.enabled = False
      #self.flythroughCollapsibleButton.enabled = False
      #self.NavigationButton.enabled = False
      self.MatlabTrackButton.text = "Stop Tracking"

      cNodes = slicer.mrmlScene.GetNodesByName('Catheter')
      if self.cNode == None:
        self.cNode = slicer.vtkMRMLIGTLConnectorNode()
        slicer.mrmlScene.AddNode(self.cNode)
        self.cNode.SetName('Catheter')
  
      self.cNode.SetType(1)
      self.cNode.SetTypeServer(18944)
      self.cNode.Start()

      if self.needleCalibrationTransform == None:
 	self.needleCalibrationTransform = slicer.vtkMRMLLinearTransformNode()
        self.needleCalibrationTransform.SetName('needleCalibrationTransform')
        slicer.mrmlScene.AddNode(self.needleCalibrationTransform)
      
      calibrationMatrix = vtk.vtkMatrix4x4()
      self.needleCalibrationTransform.GetMatrixTransformToParent(calibrationMatrix)
      calibrationMatrix.SetElement(0,0,0)
      calibrationMatrix.SetElement(0,2,1)
      calibrationMatrix.SetElement(2,0,-1)
      calibrationMatrix.SetElement(2,2,0)
      self.needleCalibrationTransform.SetMatrixTransformToParent(calibrationMatrix)

      needleModelNodes = slicer.mrmlScene.GetNodesByName('NeedleModel')
      if needleModelNodes.GetNumberOfItems() > 0:
        catheterNode = needleModelNodes.GetItemAsObject(0)
        if catheterNode.GetTransformNodeID() == None:
          catheterNode.SetAndObserveTransformNodeID(self.needleCalibrationTransform.GetID())

      cameraNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLCameraNode')
      if cameraNodes.GetNumberOfItems() > 0:
        self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
        if self.cameraForNavigation.GetTransformNodeID() == None:
          self.cameraForNavigation.SetPosition(-1.0,-0.0,0.0)
          self.cameraForNavigation.SetFocalPoint(-3.0,0.0,0.0)
          self.cameraForNavigation.SetAndObserveTransformNodeID(self.needleCalibrationTransform.GetID())

      lm = slicer.app.layoutManager()
      yellowWidget = lm.sliceWidget('Yellow')
      self.yellowLogic = yellowWidget.sliceLogic()
      redWidget = lm.sliceWidget('Red')
      self.redLogic = redWidget.sliceLogic() 
      greenWidget = lm.sliceWidget('Green')
      self.greenLogic = greenWidget.sliceLogic()

      self.sensorTimer.start()
    else:
      self.sensorTimer.stop()
      self.cNode.Stop()
      #self.cNode = None
      #self.cameraForNavigation = None
      #self.needleCalibrationTransform = None
      #self.carina = 0
      #self.CreatePathButton.enabled = True
      #self.flythroughCollapsibleButton.enabled = True
      #self.NavigationButton.enabled = True
      self.MatlabTrackButton.text = "Track Sensor"

  def ReadPosition(self):
      if self.cNode.GetState() == 2:
        transformNodes = slicer.mrmlScene.GetNodesByName('ProbeToTracker')
        tNode = transformNodes.GetItemAsObject(0)        
        if tNode:
          if self.needleCalibrationTransform.GetTransformNodeID() == None:
            self.needleCalibrationTransform.SetAndObserveTransformNodeID(tNode.GetID())
          transformMatrix = vtk.vtkMatrix4x4()
          tNode.GetMatrixTransformToParent(transformMatrix)
          x = transformMatrix.GetElement(0,3)
          y = transformMatrix.GetElement(1,3)
          z = transformMatrix.GetElement(2,3)          
          c = (x+y)/z
          viewUp = [-1,-1,c]
          self.cameraForNavigation.SetViewUp(viewUp)

          self.yellowLogic.SetSliceOffset(x)
          print self.yellowLogic.GetSliceOffset()
          self.greenLogic.SetSliceOffset(y)
          print self.greenLogic.GetSliceOffset()
          self.redLogic.SetSliceOffset(z)
          print self.redLogic.GetSliceOffset()

        '''if trackerNodes.GetNumberOfItems() > 0: 
            tracker = trackerNodes.GetItemAsObject(0)
            TransToParent = tracker.GetTransformToParent()
            Matrix = TransToParent.GetMatrix()
            print Matrix.GetElement(0,3), Matrix.GetElement(1,3), Matrix.GetElement(2,3)'''
            

      #try: 
       # self.coordinates = numpy.loadtxt("/home/acorvo/Desktop/Datasets/MatlabCode/NavigationSimulation/CenterlineSensorTracker.txt")
       # self.RealPosition.setText(self.coordinates)
      #except IOError:
       # a = numpy.zeros(3) 

      #self.fn = slicer.vtkMRMLAnnotationFiducialNode()
      #self.fn.SetFiducialWorldCoordinates((0,0,0))
      #self.fn.SetName('Sensor')   
      #slicer.mrmlScene.AddNode(self.fn)
      #self.fn.SetFiducialWorldCoordinates((self.coordinates[0],self.coordinates[1],self.coordinates[2]))

      # The sphere indicates the sensor in the 3D View
      #self.source = vtk.vtkSphereSource()
      #self.source.SetRadius(2.0)
      #self.source.SetCenter(self.coordinates[0],self.coordinates[1],self.coordinates[2])
      #self.source.Update()
      #slicer.mrmlScene.AddNode(self.track)
      #self.track.SetAndObservePolyData(self.source.GetOutput())    
                      
      #slicer.mrmlScene.AddNode(self.ModelDisplay)
      #self.track.SetAndObserveDisplayNodeID(self.ModelDisplay.GetID())  
      #slicer.modules.markups.logic().AddFiducial(self.coordinates[0],self.coordinates[1],self.coordinates[2])

  def CheckCurrentPosition(self, coords):
    if self.fiducialNode:
      p = [0,0,0]
      n = [0,0,0]
      closest = 0
      i = 0
      f = [0,0,0]
      pos = 0      
      focalPointFound = 0
      count = 0
      prevFocPoint = [0,0,0]
      prevAngle = 0
      while i < self.fiducialNode.GetNumberOfFiducials() and not(closest):
        self.fiducialNode.GetNthFiducialPosition(i,p)   
        if (abs(p[2]-coords[2]) < 1.5 and abs(p[0]-coords[0]) < 2 and abs(p[1]-coords[1]) < 3.5 ):
          for offset in range(1,20):
            self.fiducialNode.GetNthFiducialPosition(i+offset,n)
            if( abs(n[0]-coords[0]) <= abs(p[0]-coords[0]) and 
                abs(n[2]-coords[2]) <= abs(p[2]-coords[2]) and 
                abs(n[1]-coords[1]) < 2 ): 
              p[0] = n[0]
              p[1] = n[1]
              p[2] = n[2]

              pos=i+offset
             
          closest = 1
          if pos == 0:
            pos = i

          self.coordinates[0] = p[0]
          self.coordinates[1] = p[1]
          self.coordinates[2] = p[2]
          if pos == 319:
            print self.coordinates
          if self.coordinates[0] == -30.339359283447266:
            print pos
            print "closest fiducial: ", self.coordinates
        self.focalPoint = [0,0,0]
        if pos != 0:
           if pos > 100:
             j = -29
           else:
             j = 9
           while j<100 and not(focalPointFound):
            self.fiducialNode.GetNthFiducialPosition(pos+j,f)
            if( abs(f[0]-self.coordinates[0])!=0 and
                abs(f[1]-self.coordinates[1])!=0 and
                abs(f[2]-self.coordinates[2])!=0 and
                abs(f[0]-self.coordinates[0])<=5 and 
                abs(f[1]-self.coordinates[1])<=15 and
                abs(f[2]-self.coordinates[2])<=100 ):

              self.focalPoint[0] = (f[0]+self.coordinates[0])/2
              self.focalPoint[1] = (f[1]+self.coordinates[1])/2
              self.focalPoint[2] = (f[2]+self.coordinates[2])/2

              #print pos
              #print self.coordinates
              #print f

              if count == 0:
	        prevFocPoint[0] = self.focalPoint[0]
	        prevFocPoint[1] = self.focalPoint[1]
	        prevFocPoint[2] = self.focalPoint[2]
                focalPointFound = 0
              else:                              
	        ROI = [-48.740203857421875, 10.078389167785645, 1691.2109375]	      
                
                if( abs(ROI[0]-self.focalPoint[0]) <= abs(ROI[0]-prevFocPoint[0]) and
                    abs(ROI[2]-self.focalPoint[2]) <= abs(ROI[2]-prevFocPoint[2]) ):
                  prevFocPoint[0] = self.focalPoint[0]
	          prevFocPoint[1] = self.focalPoint[1]
	          prevFocPoint[2] = self.focalPoint[2]                  
                elif( abs(ROI[0]-self.focalPoint[0]) > abs(ROI[0]-prevFocPoint[0]) and
                      abs(ROI[2]-self.focalPoint[2]) <= abs(ROI[2]-prevFocPoint[2]) ):
                  prevFocPoint[0] = self.focalPoint[0]
	          prevFocPoint[1] = self.focalPoint[1]
	          prevFocPoint[2] = self.focalPoint[2]                 
                else:                                          
                  self.focalPoint[0] = prevFocPoint[0]
                  self.focalPoint[1] = prevFocPoint[1]
                  self.focalPoint[2] = prevFocPoint[2]

                if( count == 15 ):
                  focalPointFound = 1

              count += 1    

            j += 1
                    
        i += 1

      prova = self.cameraNode.GetCamera()
      prova.SetPosition(*self.coordinates)
      prova.SetFocalPoint(*self.focalPoint) 
      prevAngle = prova.GetOrientationWXYZ()[0]

  def RealPositionValueChanged(self):
    """ Apply the fth step in the path to the global camera"""
    if self.fiducialNode:
        self.fiducialNumber += 1
        #print self.fiducialNumber
        self.CheckCurrentPosition(self.coordinates) 
	f = self.coordinates
    	self.camera.SetPosition(*f)
	foc = self.focalPoint
        #fidPoint = [0,0,0]
        #nextFidPoint = [0,0,0]
        #foc = [0,0,0]
	#self.fiducialNode.GetNthFiducialPosition(self.fiducialNumber,fidPoint)
	
        #if( self.fiducialNumber+10 > self.fiducialNode.GetNumberOfFiducials() ):
        #  self.fiducialNode.GetNthFiducialPosition(self.fiducialNode.GetNumberOfFiducials()-1,nextFidPoint)
        #else:
        #  self.fiducialNode.GetNthFiducialPosition(self.fiducialNumber+10,nextFidPoint)
	       
        #foc[0] = (fidPoint[0]+nextFidPoint[0])/2
        #foc[1] = (fidPoint[1]+nextFidPoint[1])/2
        #foc[2] = (fidPoint[2]+nextFidPoint[2])/2

	self.camera.SetFocalPoint(*foc)
 
        toParent = vtk.vtkMatrix4x4()
        self.transform.GetMatrixTransformToParent(toParent)
        toParent.SetElement(0, 3, f[0])
        toParent.SetElement(1, 3, f[1])
        toParent.SetElement(2, 3, f[2])
        self.transform.SetMatrixTransformToParent(toParent)
    
class BronchoscopyComputePath:
  """Compute path given a list of fiducials. 
  A Hermite spline interpolation is used. See http://en.wikipedia.org/wiki/Cubic_Hermite_spline
  
  Example:
    result = BronchoscopyComputePath(fiducialListNode)
    print "computer path has %d elements" % len(result.path)
    
  """
  
  def __init__(self, fiducialListNode, dl = 0.5):
    
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

    self.path = [self.p[0]]
    self.calculatePath()

  def calculatePath(self):
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
      t, p, remainder = self.step(segment, t, self.dl)
      if remainder != 0 or t == 1.:
        segment += 1
        t = 0
        if segment < n-1:
          t, p, remainder = self.step(segment, t, remainder)
      self.path.append(p)

  def point(self,segment,t):
    return (self.h00(t)*self.p[segment] + 
              self.h10(t)*self.m[segment] + 
              self.h01(t)*self.p[segment+1] + 
              self.h11(t)*self.m[segment+1])

  def step(self,segment,t,dl):
    """ Take a step of dl and return the path point and new t
      return:
      t = new parametric coordinate after step 
      p = point after step
      remainder = if step results in parametic coordinate > 1.0, then
        this is the amount of world space not covered by step
    """
    import numpy.linalg
    p0 = self.path[self.path.__len__() - 1] # last element in path
    remainder = 0
    ratio = 100
    count = 0
    while abs(1. - ratio) > 0.05:
      t1 = t + self.dt
      pguess = self.point(segment,t1)
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
      p1 = self.point(segment, t1)
      remainder = numpy.linalg.norm(p1 - pguess)
      pguess = p1
    return (t1, pguess, remainder)
