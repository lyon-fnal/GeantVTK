#------------------------------------------------------------------------------
# sources.cmake
# Module : G4visVTK
# Package: Geant4.src.G4visualization.G4visVTK
#
# Sources description for a library.
# Lists the sources and headers of the code explicitely.
# Lists include paths needed.
# Lists the internal granular and global dependencies of the library.
# Source specific properties should be added at the end.
#
# Generated on : 24/9/2010
#
# $Id: sources.cmake 88190 2015-02-02 17:24:54Z gcosmo $
#
#------------------------------------------------------------------------------



# List external includes needed.
include_directories(${CLHEP_INCLUDE_DIRS})
include_directories(${USOLIDS_INCLUDE_DIRS})
include_directories(${PARAVIEW_INCLUDE_DIRS})
include_directories(${VTK_INCLUDE_DIRS})

#link_directories(${VTK_LIBRARY_DIRS})

# List internal includes needed.
include_directories(${CMAKE_SOURCE_DIR}/source/digits_hits/hits/include)
include_directories(${CMAKE_SOURCE_DIR}/source/geometry/management/include)
include_directories(${CMAKE_SOURCE_DIR}/source/geometry/solids/CSG/include)
include_directories(${CMAKE_SOURCE_DIR}/source/geometry/solids/specific/include)
include_directories(${CMAKE_SOURCE_DIR}/source/global/HEPGeometry/include)
include_directories(${CMAKE_SOURCE_DIR}/source/global/management/include)
include_directories(${CMAKE_SOURCE_DIR}/source/graphics_reps/include)
include_directories(${CMAKE_SOURCE_DIR}/source/intercoms/include)
include_directories(${CMAKE_SOURCE_DIR}/source/tracking/include)
include_directories(${CMAKE_SOURCE_DIR}/source/visualization/management/include)
include_directories(${CMAKE_SOURCE_DIR}/source/visualization/modeling/include)

set(vtklibs "${VTK_LIBRARIES}")
#list(REMOVE_ITEM vtklibs vtkPVPythonCatalyst vtkUtilitiesPythonInitializer)

add_definitions("-DG4VIS_BUILD_VTK_DRIVER")
add_definitions("-DUSE_CATALYST")

message( "VTK LIBRARIES" )
message( "${VTK_LIBRARIES}")

message( "VTK_LIBRARY_DIRS" )
message( "${VTK_LIBRARY_DIRS}" )
#
# Define the Geant4 Module.
#
include(Geant4MacroDefineModule)
GEANT4_DEFINE_MODULE(NAME G4visVTK
    HEADERS
        G4VTK.hh
        G4VTKSceneHandler.hh
        G4VTKViewer.hh
    SOURCES
        G4VTK.cc
        G4VTKSceneHandler.cc
        G4VTKViewer.cc
    GRANULAR_DEPENDENCIES
        G4csg
        G4geometrymng
        G4globman
        G4graphics_reps
        G4hits
        G4intercoms
        G4modeling
        G4specsolids
        G4tracking
        G4vis_management
    GLOBAL_DEPENDENCIES
        G4digits_hits
        G4geometry
        G4global
        G4graphics_reps
        G4intercoms
        G4modeling
        G4tracking
        G4vis_management
    LINK_LIBRARIES
	${vtklibs}
)

# List any source specific properties here

