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

    self.fiducialNode = None
    self.path = None

    self.pathCreated = 0
    
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
 
    ###################################################################################
    #########################  Extract Centerline Button  #############################
    ###################################################################################
    self.ExtractCenterlineButton = qt.QPushButton("Extract Centerline")
    self.ExtractCenterlineButton.toolTip = "Run the algorithm to extract centerline of the model on which fiducials will be placed. Fiducials are necessary to compensate for possible registration issues."
    self.ExtractCenterlineButton.setFixedSize(200,50)
    if self.inputSelector.currentNode() and self.labelSelector.currentNode():
        self.ExtractCenterlineButton.enabled = True
    else:
        self.ExtractCenterlineButton.enabled = False
    IOFormLayout.addWidget(self.ExtractCenterlineButton)

    ####################################################################################
    #### Optional Collapsible Button To Select An Uploaded Centerline Fiducials List ###
    ####################################################################################
    self.fiducialsCollapsibleButton = ctk.ctkCollapsibleButton()
    self.fiducialsCollapsibleButton.text = "Centerline Fiducials List"
    self.fiducialsCollapsibleButton.setChecked(False)
    #self.fiducialsCollapsibleButton.setFixedSize(400,40)
    self.fiducialsCollapsibleButton.enabled = True
    self.layout.addWidget(self.fiducialsCollapsibleButton)
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

    ####################################################################################
    ######################  Create Path Towards An ROI Section  ########################
    ####################################################################################
    self.pathCreationCollapsibleButton = ctk.ctkCollapsibleButton()
    self.pathCreationCollapsibleButton.text = "Path Creation"
    self.pathCreationCollapsibleButton.setChecked(True)
    #self.fiducialsCollapsibleButton.setFixedSize(400,40)
    self.pathCreationCollapsibleButton.enabled = True
    self.layout.addWidget(self.pathCreationCollapsibleButton)
    pathCreationFormLayout = qt.QFormLayout(self.pathCreationCollapsibleButton)

    self.pointsListSelector = slicer.qMRMLNodeComboBox()
    self.pointsListSelector.nodeTypes = ( ("vtkMRMLMarkupsFiducialNode"), "" )
    self.pointsListSelector.selectNodeUponCreation = True
    self.pointsListSelector.addEnabled = False
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
    self.PathCreationButton = qt.QPushButton("Create Path")
    self.PathCreationButton.toolTip = "Run the algorithm to create the path between the specified points."
    self.PathCreationButton.setFixedSize(300,50)
    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode():
        self.PathCreationButton.enabled = True
    else:
        self.PathCreationButton.enabled = False
    pathCreationFormLayout.addWidget(self.PathCreationButton)

    #############################################################################################
    ###########################  Sensor Tracker Collapsible Button  #############################
    #############################################################################################

    trackerCollapsibleButton = ctk.ctkCollapsibleButton()
    trackerCollapsibleButton.text = "Probe Tracking"
    self.layout.addWidget(trackerCollapsibleButton)
    self.layout.setSpacing(20)
    trackerFormLayout = qt.QFormLayout(trackerCollapsibleButton)

    ##############################################################################################
    ##############################  Matlab/Probe Track Button  ##################################
    ##############################################################################################
    self.ProbeTrackButton = qt.QPushButton("Track Sensor")
    self.ProbeTrackButton.toolTip = "Track sensor output."
    #self.ProbeTrackButton.setFixedSize(300,40)
    self.ProbeTrackButton.setFixedHeight(40)
    self.ProbeTrackButton.checkable = True
    if self.fiducialListSelector.currentNode():
        self.ProbeTrackButton.enabled = True
    else:
        self.ProbeTrackButton.enabled = False
    
    trackerFormLayout.addWidget(self.ProbeTrackButton)

    #
    # Create Connections
    #
    self.ExtractCenterlineButton.connect('clicked(bool)', self.onExtractCenterlineButton)
    self.inputSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.labelSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.fiducialListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.pointsListSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
    self.PathCreationButton.connect('clicked(bool)', self.onPathCreationButton)
    self.ProbeTrackButton.connect('toggled(bool)', self.onProbeTrackButtonToggled)
    
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
    if self.inputSelector.currentNode() and self.labelSelector.currentNode():
      inputVolume = self.inputSelector.currentNode()
      modelDisplayNode = inputVolume.GetDisplayNode()
      modelDisplayNode.SetColor(1.0, 0.8, 0.7)
      modelDisplayNode.SetFrontfaceCulling(1)
      modelDisplayNode.SetBackfaceCulling(0)

      self.updateGUI()
      self.ExtractCenterlineButton.enabled = True
      if self.fiducialListSelector.currentNode():
        self.ProbeTrackButton.enabled = True
    else:
      self.ExtractCenterlineButton.enabled = False
      self.PathCreationButton.enabled = False
      self.ProbeTrackButton.enabled = False

    if self.inputSelector.currentNode() and self.pointsListSelector.currentNode():
       self.PathCreationButton.enabled = True
    else:
       self.PathCreationButton.enabled = False

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

################################### Extract Centerline ######################################## 

  def onExtractCenterlineButton(self):
    # Disable Buttons 
    self.ExtractCenterlineButton.enabled = False
    self.ProbeTrackButton.enabled = False
    self.inputSelector.enabled = False
    self.labelSelector.enabled = False
    self.fiducialListSelector.enabled = False
    
    # Extract Centerline 
    self.extractCenterline(self.labelSelector.currentNode()) 
    
    # Enable Buttons Again
    self.ExtractCenterlineButton.enabled = True
    self.ProbeTrackButton.enabled = True
    self.inputSelector.enabled = True
    self.labelSelector.enabled = True
    self.fiducialListSelector.enabled = True

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
      self.centerlineModel.SetDisplayVisibility(0)

      # create fiducial list
      self.fiducialNode = slicer.vtkMRMLMarkupsFiducialNode()
      self.fiducialNode.SetName('CenterlineFiducials')
      slicer.mrmlScene.AddNode(self.fiducialNode)

      centerlinePolydata = self.centerlineModel.GetPolyData()
      self.points = vtk.vtkPoints()

      iterations = 5
      self.Smoothing(centerlinePolydata, self.points, iterations)
      self.CreateFiducialsCenterline(self.points, self.fiducialNode)

    self.fiducialNode.SetDisplayVisibility(0)
    self.ProbeTrackButton.enabled = True

    return True

  def CreateFiducialsCenterline(self, centerlinePoints, fiducialList):

    NoP = centerlinePoints.GetNumberOfPoints()
    
    #NthFiducial = 0
    point = [0,0,0]

    for i in range(0, NoP):
      centerlinePoints.GetPoint(i,point)
      fiducialList.AddFiducial(point[0],point[1],point[2])
      #fiducialList.SetNthMarkupVisibility(NthFiducial,0)
      #NthFiducial += 1

    fiducialList.SetDisplayVisibility(0)

  def Smoothing(self, centModel, modelPoints, iterationsNumber):
    
    NumberOfCells = centModel.GetNumberOfCells()

    pointsList = []
    distancePointsAbove = []
    distancePointsBelow = []

    for iteration in range(0, iterationsNumber):
      if iteration == 0:
        centralPoint = [0,0,0]
        for i in range(NumberOfCells-10,10,-10):
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

################################### Create Path Between Points ######################################## 

  def onPathCreationButton(self):
    # Disable Buttons
    self.pathCreated = 1
    self.ExtractCenterlineButton.enabled = False
    self.PathCreationButton.enabled = False
    self.ProbeTrackButton.enabled = False
    self.inputSelector.enabled = False
    self.labelSelector.enabled = False
    self.fiducialListSelector.enabled = False
    
    # Create Centerline Path 
    self.pathComputation(self.inputSelector.currentNode(), self.pointsListSelector.currentNode()) 
    
    # Enable Buttons Again
    self.ExtractCenterlineButton.enabled = True
    self.PathCreationButton.enabled = True
    self.ProbeTrackButton.enabled = True
    self.inputSelector.enabled = True
    self.labelSelector.enabled = True
    self.fiducialListSelector.enabled = True

    # Update GUI
    self.updateGUI()

  def pathComputation(self, inputModel, points):
    """
    Run the actual algorithm to create the path between the 2 fiducials
    """
    import vtkSlicerCenterlineExtractionModuleLogic

    if( points.GetNumberOfMarkups() > 10 ):
      return False
    
    '''modelNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLModelNode')
    for n in xrange( modelNodes.GetNumberOfItems() ):
      node = modelNodes.GetItemAsObject(n)
      nodeName = node.GetName()
      replaceName = nodeName.replace('-', ' - ')
      splitName = replaceName.split()
      for item in splitName:
        if (item == 'Path') or (item == 'Cursor') or (item == 'Transform'):
          slicer.mrmlScene.RemoveNode(node)'''
                 
    inputPolyData = inputModel.GetPolyData()

    sourcePosition = [0,0,0]
    points.GetNthFiducialPosition(0,sourcePosition)

    source = inputPolyData.FindPoint(sourcePosition)

    sourceId = vtk.vtkIdList()
    sourceId.SetNumberOfIds(1) 
    sourceId.InsertId(0,source)

    targetPosition = [0,0,0]
    targetId = vtk.vtkIdList()
    targetId.SetNumberOfIds(points.GetNumberOfFiducials()-1) 
      
    for i in range(1, points.GetNumberOfFiducials()):
      points.GetNthFiducialPosition(i,targetPosition)
 
      target = inputPolyData.FindPoint(targetPosition)
      targetId.InsertId(i-1,target)

    pathCreation = vtkSlicerCenterlineExtractionModuleLogic.vtkvmtkPolyDataCenterlines()

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

      self.pathFiducialsNode = slicer.vtkMRMLMarkupsFiducialNode()
      self.pathFiducialsNode.SetName('pathFiducials')
      slicer.mrmlScene.AddNode(self.pathFiducialsNode)

      self.CreateFiducialsPath(self.createdPath, self.pathFiducialsNode)
      model = BronchoscopyPathModel(self.pathFiducialsNode)
      
      #slicer.mrmlScene.RemoveNode(self.pathFiducialsNode)
      markupLogic = slicer.modules.markups.logic()
      markupLogic.SetActiveListID(points)
      #self.pathFiducialsNode = None
      #self.createdPath = None

    return True
     
  def pathSmoothing(self, pathModel):
      
    import vtkSlicerCenterlineExtractionModuleLogic
    
    NumberOfPoints = pathModel.GetNumberOfPoints()
    position = NumberOfPoints-1
    startingPoint = [0,0,0]
    pathModel.GetPoint(position,startingPoint)
    #print "Starting point centerline: ",startingPoint
    targetPosition=[0,0,0]
    pathModel.GetPoint(1,targetPosition)
           
    squaredDist = vtk.vtkMath.Distance2BetweenPoints(startingPoint,targetPosition)
    #print squaredDist 
    
    if (squaredDist < 10.000) :
      smoothfactor = 1
      iterations = 1000
    else:
      smoothfactor = 1
      iterations = 100
      
    centerlineSmoothing = vtkSlicerCenterlineExtractionModuleLogic.vtkvmtkCenterlineSmoothing()
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

    fiducialList.SetDisplayVisibility(0)

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
  


############################# SENSOR TRACKING #################################

  def onProbeTrackButtonToggled(self, checked):     
    if checked:
      if self.fiducialListSelector.currentNode():
        self.fiducialNode = self.fiducialListSelector.currentNode()
        self.fiducialNode.SetDisplayVisibility(0)

      self.ExtractCenterlineButton.enabled = False
      self.inputSelector.enabled = False
      self.labelSelector.enabled = False
      self.fiducialListSelector.enabled = False
      self.PathCreationButton.enabled = False

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

      ################## This turns the probe of 90 degrees #####################

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

        ########### A fiducial is created to indicate the position of the probe in saggital, coronal and axial views #########

        #probePositionIndicator = slicer.vtkMRMLMarkupsFiducialNode()
        #probePositionIndicator.SetName('ProbePositionIndicator')
        #slicer.mrmlScene.AddNode(probePositionIndicator)
        # The fiducial is placed on the tip of the probe
        #probePositionIndicator.AddFiducial(-1.0,-0.0,0.0)
        #probeIndicatorDisplayNode = probePositionIndicator.GetDisplayNode()
        #probeIndicatorDisplayNode.SetGlyphScale(6.0)
        #probeIndicatorDisplayNode.SetGlyphType(10)
        #probeIndicatorDisplayNode.SetTextScale(0.0)

        if probeNode.GetTransformNodeID() == None:
          probeNode.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
          #probePositionIndicator.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())

      ################## Camera is connected to the transform #####################

      cameraNodes = slicer.mrmlScene.GetNodesByName('PerspexCamera')
      if cameraNodes.GetNumberOfItems() > 0:
        self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
        if self.cameraForNavigation.GetTransformNodeID() == None:
          self.cameraForNavigation.SetPosition(-1.0,-0.0,0.0)
          self.cameraForNavigation.SetFocalPoint(-6.0,0.0,0.0)
          self.cameraForNavigation.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
      else:
        cameraNodes = slicer.mrmlScene.GetNodesByClass('vtkMRMLCameraNode')
	self.cameraForNavigation = cameraNodes.GetItemAsObject(0)
        if cameraNodes.GetNumberOfItems() > 0:
          if self.cameraForNavigation.GetTransformNodeID() == None:
            self.cameraForNavigation.SetPosition(-1.0,-0.0,0.0)
            self.cameraForNavigation.SetFocalPoint(-6.0,0.0,0.0)
            self.cameraForNavigation.SetAndObserveTransformNodeID(self.probeCalibrationTransform.GetID())
          else:
            self.cameraForNavigation.SetPosition(-1.0,-0.0,0.0)
            self.cameraForNavigation.SetFocalPoint(-6.0,0.0,0.0)

      lm = slicer.app.layoutManager()
      yellowWidget = lm.sliceWidget('Yellow')
      self.yellowLogic = yellowWidget.sliceLogic()
      redWidget = lm.sliceWidget('Red')
      self.redLogic = redWidget.sliceLogic() 
      greenWidget = lm.sliceWidget('Green')
      self.greenLogic = greenWidget.sliceLogic()

      self.sensorTimer.start()
       
    else:  # When button is released...
      self.sensorTimer.stop()
      self.cNode.Stop()
      #self.cNode = None
      #self.cameraForNavigation = None
      #self.probeCalibrationTransform = None

      self.ExtractCenterlineButton.enabled = True
      self.inputSelector.enabled = True
      self.labelSelector.enabled = True
      self.fiducialListSelector.enabled = True
      self.PathCreationButton.enabled = True

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

  def CheckCurrentPosition(self, tMatrix):

    fiducialPos = [0,0,0]
    fiducialsList = []
    distance = []

    if self.fiducialNode == None:
      #fiducialNodesCollection = slicer.mrmlScene.GetNodesByName('CenterlineFiducials_Testing')
      fiducialNodesCollection = slicer.mrmlScene.GetNodesByName('CenterlineFiducials')
      self.fiducialNode = fiducialNodesCollection.GetItemAsObject(0)

    if self.fiducialNode:
      for i in xrange(self.fiducialNode.GetNumberOfFiducials()):
        self.fiducialNode.GetNthFiducialPosition(i,fiducialPos)
        s = [fiducialPos[0],fiducialPos[1],fiducialPos[2]]
        fiducialsList.append(s)

    if self.pathCreated == 1:
      pathFiducialsCollection = slicer.mrmlScene.GetNodesByName('pathFiducials')
      for j in xrange(pathFiducialsCollection.GetNumberOfItems()):
        fidNode = pathFiducialsCollection.GetItemAsObject(j)
        for n in xrange(fidNode.GetNumberOfFiducials()):
          fidNode.GetNthFiducialPosition(n,fiducialPos)
          s = [fiducialPos[0],fiducialPos[1],fiducialPos[2]]
          fiducialsList.append(s)

    originalCoord = [0,0,0]
    originalCoord[0] = tMatrix.GetElement(0,3)
    originalCoord[1] = tMatrix.GetElement(1,3)
    originalCoord[2] = tMatrix.GetElement(2,3)  

    originalCoord = numpy.asarray(originalCoord)

    distance = ((fiducialsList-originalCoord)**2).sum(axis=1)
    ndx = distance.argsort()
    closestPoint = fiducialsList[ndx[0]]

    tMatrix.SetElement(0,3,closestPoint[0])
    tMatrix.SetElement(1,3,closestPoint[1])
    tMatrix.SetElement(2,3,closestPoint[2])

    ####################################################################################################################
    # Continuosly Update ViewUp Of The Camera To Always Have It On The Direction Orthogonal To The Locator's Long Axis #
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

class BronchoscopyPathModel:
  """Create a vtkPolyData for a polyline:
       - Add one point per path point.
       - Add a single polyline
  """
  def __init__(self, fiducialListNode):
  
    fids = fiducialListNode
    scene = slicer.mrmlScene
    
    points = vtk.vtkPoints()
    self.polyData = vtk.vtkPolyData()
    self.polyData.SetPoints(points)

    lines = vtk.vtkCellArray()
    self.polyData.SetLines(lines)
    linesIDArray = lines.GetData()
    linesIDArray.Reset()
    linesIDArray.InsertNextTuple1(0)

    polygons = vtk.vtkCellArray()
    self.polyData.SetPolys( polygons )
    idArray = polygons.GetData()
    idArray.Reset()
    idArray.InsertNextTuple1(0)

    """for point in path:
      pointIndex = points.InsertNextPoint(*point)
      linesIDArray.InsertNextTuple1(pointIndex)
      linesIDArray.SetTuple1( 0, linesIDArray.GetNumberOfTuples() - 1 )
      lines.SetNumberOfCells(1)"""
    index = [0,0,0]
 
    for n in xrange(0,fiducialListNode.GetNumberOfFiducials()):
      fiducialListNode.GetNthFiducialPosition(n,index)
      pointIndex = points.InsertNextPoint(index)
      linesIDArray.InsertNextTuple1(pointIndex)
      linesIDArray.SetTuple1( 0, linesIDArray.GetNumberOfTuples() - 1 )
      lines.SetNumberOfCells(1)

    #self.pathSmoothing(self.polyData)

    # Create model node
    model = slicer.vtkMRMLModelNode()
    model.SetScene(scene)
    model.SetName(scene.GenerateUniqueName("Path-%s" % fids.GetName()))
    model.SetAndObservePolyData(self.polyData)

    # Create display node
    modelDisplay = slicer.vtkMRMLModelDisplayNode()
    modelDisplay.SetColor(1,1,0) # yellow
    modelDisplay.SetScene(scene)
    scene.AddNode(modelDisplay)
    model.SetAndObserveDisplayNodeID(modelDisplay.GetID())

    # Add to scene
    if vtk.VTK_MAJOR_VERSION <= 5:
      # shall not be needed.
      modelDisplay.SetInputPolyData(model.GetPolyData())
    scene.AddNode(model)
