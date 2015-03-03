cmake_minimum_required(VERSION 2.8.9)

project(BronchoscopyScripted)

#-----------------------------------------------------------------------------
set(EXTENSION_HOMEPAGE "http://www.slicer.org/slicerWiki/index.php/Documentation/4.3/Extensions/BronchoscopyScripted")
set(EXTENSION_CATEGORY "Endoscopy")
set(EXTENSION_CONTRIBUTORS "Pietro Nardelli & Alberto Corvo (University College Cork)")
set(EXTENSION_DESCRIPTION "Scripted loadable module for centerline extraction and virtual navigation")
set(EXTENSION_ICONURL "http://viewvc.slicer.org/viewvc.cgi/Slicer4/trunk/Extensions/Testing/BronchoscopyScripted/BronchoscopyScripted.png?revision=21747&view=co")
set(EXTENSION_SCREENSHOTURLS "http://wiki.slicer.org/slicerWiki/images/e/e2/Slicer-r19441-BronchoscopyScripted-screenshot.png")

#-----------------------------------------------------------------------------
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
add_subdirectory(Bronchoscopy)

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_CPACK})
