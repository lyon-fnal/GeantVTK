//
// ********************************************************************
// * License and Disclaimer                                           *
// *                                                                  *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// *                                                                  *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.         *
// *                                                                  *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
//
// $Id: G4VTKSceneHandler.cc 66870 2013-01-14 23:38:59Z adotti $
//
// 
// Owen Crawford 13th June 2016
// A driver for writing VTK files.
// Modified from XXX template files.

#include "G4VTKSceneHandler.hh"

#include "G4VTKViewer.hh"
#include "G4PhysicalVolumeModel.hh"
#include "G4LogicalVolumeModel.hh"
#include "G4VPhysicalVolume.hh"
#include "G4LogicalVolume.hh"
#include "G4Box.hh"
#include "G4Polyline.hh"
#include "G4Text.hh"
#include "G4Circle.hh"
#include "G4Square.hh"
#include "G4Polyhedron.hh"
#include "G4UnitsTable.hh"
#include "G4Point3D.hh"

#include "vtkRegularPolygonSource.h"
#include "vtkPolyhedron.h"
#include "vtkCubeSource.h"
#include "vtkPolygon.h"

#include <sstream>

//#include <vtkVersion.h>
//#include <vtkSmartPointer.h>
//#include <vtkProperty.h>
//#include <vtkXMLImageDataWriter.h>
//#include <vtkXMLImageDataReader.h>
//#include <vtkImageData.h>
//#include <vtkPolyData.h>
//#include <vtkRenderWindowInteractor.h>
//#include <vtkRenderer.h>
//#include <vtkImageDataGeometryFilter.h>
//#include <vtkRenderWindow.h>
//#include <vtkActor.h>
//#include <vtkXMLPolyDataWriter.h>
//#include <vtkShortArray.h>
//#include <vtkPointData.h>
//#include <vtkCellData.h>
//#include <vtkCellArray.h>
//#include <vtkStringArray.h>
//#include <vtkIntArray.h>
//#include <vtkDoubleArray.h>
//#include <vtkPoints.h>
//#include <vtkPolyLine.h>
//#include <vtkAppendPolyData.h>

G4int G4VTKSceneHandler::fSceneIdCount = 0;
// Counter for VTK scene handlers.


G4VTKSceneHandler::G4VTKSceneHandler(G4VGraphicsSystem& system,
					     const G4String& name):
  G4VSceneHandler(system, fSceneIdCount++, name)
{
    rootMBDS = vtkSmartPointer<vtkMultiBlockDataSet>::New();
    leafMBDS = vtkSmartPointer<vtkMultiBlockDataSet>::New();
}

G4VTKSceneHandler::~G4VTKSceneHandler() {}

void G4VTKSceneHandler::BeginModeling()
{
    
}
void G4VTKSceneHandler::EndModeling()
{
    dynamic_cast<G4VTKViewer*>(fpViewer)->
        GetFileWriter().WriteItem(rootMBDS);
}

#ifdef G4VTKDEBUG
// Useful function...
void G4VTKSceneHandler::PrintThings() {
  G4cout <<
    "  with transformation "
         << (void*)fpObjectTransformation;
  if (fpModel) {
    G4cout << " from " << fpModel->GetCurrentDescription()
	   << " (tag " << fpModel->GetCurrentTag()
	   << ')';
  } else {
    G4cout << "(not from a model)";
  }
  G4PhysicalVolumeModel* pPVModel =
    dynamic_cast<G4PhysicalVolumeModel*>(fpModel);
  if (pPVModel) {
    G4cout <<
      "\n  current physical volume: "
           << pPVModel->GetCurrentPV()->GetName() <<
      "\n  current logical volume: "
// There might be a problem with the LV pointer if this is a G4LogicalVolumeModel
           << pPVModel->GetCurrentLV()->GetName() <<
      "\n  current depth of geometry tree: "
           << pPVModel->GetCurrentDepth();
  }
  G4cout << G4endl;
}
#endif

// Note: This function overrides G4VSceneHandler::AddSolid(const
// G4Box&).  You may not want to do this, but this is how it's done if
// you do.  Certain other specific solids may be treated this way -
// see G4VSceneHandler.hh.  The simplest possible driver would *not*
// implement these polymorphic functions, with the effect that the
// default versions in G4VSceneHandler are used, which simply call
// G4VSceneHandler::RequestPrimitives to turn the solid into a
// G4Polyhedron usually.
void G4VTKSceneHandler::AddSolid(const G4Box& box) {
#ifdef G4VTKDEBUG
  G4cout <<
    "G4VTKSceneHandler::AddSolid(const G4Box& box) called for "
	 << box.GetName()
	 << G4endl;
#endif
  //?? Process your box...
  
    //TODO
  
  G4VSceneHandler::AddSolid(box);
  
//  std::ostringstream oss;
//  oss << "G4Box(" <<
//    G4String
//    (G4BestUnit
//     (G4ThreeVector
//      (box.GetXHalfLength(), box.GetYHalfLength(), box.GetZHalfLength()),
//      "Length")).strip() << ')';
//  dynamic_cast<G4VTKViewer*>(fpViewer)->
//    GetFileWriter().WriteItem(oss.str());
}

void G4VTKSceneHandler::AddPrimitive(const G4Polyline& polyline) {
    
#ifdef G4VTKDEBUG
  G4cout <<
    "G4VTKSceneHandler::AddPrimitive(const G4Polyline& polyline) called.\n"
	 << polyline
	 << G4endl;
#endif
  // Get vis attributes - pick up defaults if none.
  const G4VisAttributes* pVA =
    fpViewer -> GetApplicableVisAttributes (polyline.GetVisAttributes ());
  //?? Process polyline
  
  if(pVA && pVA->IsVisible()){
      
    vtkSmartPointer<vtkPolyLine> vtkpl = vtkSmartPointer<vtkPolyLine>::New();
    vtkpl->GetPointIds()->SetNumberOfIds(polyline.size());
    vtkSmartPointer<vtkPoints> vtkpts = vtkSmartPointer<vtkPoints>::New();
    vtkpts->SetNumberOfPoints(polyline.size());
    
    for (size_t i=0; i < polyline.size(); i++) {
	      G4Point3D vertex = polyline[i];
        int id = vtkpts->InsertNextPoint(vertex.x(), vertex.y(), vertex.z());
        vtkpl->GetPointIds()->SetId(i, id);
    }

    vtkSmartPointer<vtkCellArray> ca = vtkSmartPointer<vtkCellArray>::New();
    ca->InsertNextCell(vtkpl);
    vtkSmartPointer<vtkPolyData> vtkpd = vtkSmartPointer<vtkPolyData>::New();
    vtkpd->SetPoints(vtkpts);
    vtkpd->SetLines(ca);
    
    leafMBDS->SetBlock(leafMBDS->GetNumberOfBlocks(), vtkpd);
  }
  else{
      std::cout<<"  Ignoring invisible polyline"<<std::endl;
  }
  
}

void G4VTKSceneHandler::AddPrimitive(const G4Text& text) {
#ifdef G4VTKDEBUG
  G4cout <<
    "G4VTKSceneHandler::AddPrimitive(const G4Text& text) called.\n"
	 << text
	 << G4endl;
#endif
  // Get text colour - special method since default text colour is
  // determined by the default text vis attributes, which may be
  // specified independent of default vis attributes of other types of
  // visible objects.
  //const G4Colour& c = GetTextColour (text);  // Picks up default if none.
  //?? Process text.
  
  std::cout<<"Text not implemented yet!"<<std::endl;
  //std::cout<<"    "<<text<<std::endl;
    //TODO
//  std::ostringstream oss;
//  oss << text;
//  dynamic_cast<G4VTKViewer*>(fpViewer)->
//    GetFileWriter().WriteItem(oss.str());
}

void G4VTKSceneHandler::AddPrimitive(const G4Circle& circle) {
#ifdef G4VTKDEBUG
  G4cout <<
    "G4VTKSceneHandler::AddPrimitive(const G4Circle& circle) called.\n  "
	 << circle
	 << G4endl;
  MarkerSizeType sizeType;
  G4double size = GetMarkerSize (circle, sizeType);
  switch (sizeType) {
  default:
  case screen:
    // Draw in screen coordinates.
    G4cout << "screen";
    break;
  case world:
    // Draw in world coordinates.
    G4cout << "world";
    break;
  }
  G4cout << " size: " << size << G4endl;
#endif
  // Get vis attributes - pick up defaults if none.
  //const G4VisAttributes* pVA =
  //  fpViewer -> GetApplicableVisAttributes (circle.GetVisAttributes ());
  //?? Process circle.

  std::cout<<"Creating Circle..."<<std::endl;
  
  vtkSmartPointer<vtkRegularPolygonSource> polySrc
          = vtkSmartPointer<vtkRegularPolygonSource>::New();
  G4Point3D ctr = circle.GetPosition();
  polySrc->SetCenter(ctr.x(), ctr.y(), ctr.z());
  polySrc->SetRadius(circle.GetWorldRadius());
  //vtkCircleSrc->SetNormal(0.0, 0.0, 1.0); //Should get normal from the G4Circle
  polySrc->SetNumberOfSides(128); //Does Geant supply a constant for the number of sides on a circle??
  polySrc->Update();
  leafMBDS->SetBlock(leafMBDS->GetNumberOfBlocks(), polySrc->GetOutput());
}

void G4VTKSceneHandler::AddPrimitive(const G4Square& square) {
#ifdef G4VTKDEBUG
  G4cout <<
    "G4VTKSceneHandler::AddPrimitive(const G4Square& square) called.\n"
	 << square
	 << G4endl;
  MarkerSizeType sizeType;
  G4double size = GetMarkerSize (square, sizeType);
  switch (sizeType) {
  default:
  case screen:
    // Draw in screen coordinates.
    G4cout << "screen";
    break;
  case world:
    // Draw in world coordinates.
    G4cout << "world";
    break;
  }
  G4cout << " size: " << size << G4endl;
#endif
  // Get vis attributes - pick up defaults if none.
  //const G4VisAttributes* pVA =
  //  fpViewer -> GetApplicableVisAttributes (square.GetVisAttributes ());
  //?? Process square.
  
  std::cout<<"Creating Square..."<<std::endl;
  
  vtkSmartPointer<vtkRegularPolygonSource> polySrc
            = vtkSmartPointer<vtkRegularPolygonSource>::New();
  G4Point3D ctr = square.GetPosition();
  polySrc->SetCenter(ctr.x(), ctr.y(), ctr.z());
  polySrc->SetRadius(square.GetWorldRadius());
  //polySrc->SetNormal(0.0, 0.0, 1.0); //Should get normal from the G4Square
  polySrc->SetNumberOfSides(4);
  polySrc->Update();
  leafMBDS->SetBlock(leafMBDS->GetNumberOfBlocks(), polySrc->GetOutput());
}

void G4VTKSceneHandler::AddPrimitive(const G4Polyhedron& polyhedron) {
#ifdef G4VTKDEBUG
  G4cout <<
"G4VTKSceneHandler::AddPrimitive(const G4Polyhedron& polyhedron) called.\n"
	 << polyhedron
	 << G4endl;
#endif
  //?? Process polyhedron.
  
    G4Normal3D surfaceNormal;
    G4Point3D vertex;

    if (polyhedron.GetNoFacets()==0) return;
	
    G4int currentDepth = 0;
    G4PhysicalVolumeModel* pPVModel =
      dynamic_cast<G4PhysicalVolumeModel*>(fpModel);
    if (pPVModel) currentDepth = pPVModel->GetCurrentDepth();
    
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
    pts->SetNumberOfPoints(polyhedron.GetNoVertices());
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
                //each face is a cell
    
    G4bool notLastFace;
    do { //loop through faces
        notLastFace = polyhedron.GetNextNormal (surfaceNormal);
        vtkSmartPointer<vtkPolygon> face = vtkSmartPointer<vtkPolygon>::New();
        
        //had to be set to something.
        //the InsertId() below will resize it
        face->GetPointIds()->SetNumberOfIds(1); 
        
        G4int edgeFlag = 1;
        G4bool notLastEdge;
        int i=0;
        do { //loop through vertices on the edge
            notLastEdge = polyhedron.GetNextVertex (vertex, edgeFlag);
            int id = pts->InsertNextPoint(vertex);
            face->GetPointIds()->InsertId(i, id);
            i++;
        } while (notLastEdge);
        
        cells->InsertNextCell(face);
    } while (notLastFace);

    vtkSmartPointer<vtkPolyData> pd = vtkSmartPointer<vtkPolyData>::New();
    pd->SetPoints(pts);
    pd->SetLines(cells);
    leafMBDS->SetBlock(leafMBDS->GetNumberOfBlocks(), pd);
}

void G4VTKSceneHandler::PreAddSolid
(const G4Transform3D& objectTransformation, const G4VisAttributes& visAttribs)
{    
    //
    //This function copied from G4VTreeSceneHandler.cc
    //
    
    std::cout<<"PreAddSolid Called"<<std::endl;
  G4VSceneHandler::PreAddSolid (objectTransformation, visAttribs);
  
  G4PhysicalVolumeModel* pPVModel =
    dynamic_cast<G4PhysicalVolumeModel*>(fpModel);
  if (!pPVModel) return;  // Not from a G4PhysicalVolumeModel.

  // This call comes from a G4PhysicalVolumeModel, drawnPVPath is
  // the path of the current drawn (non-culled) volume in terms of
  // drawn (non-culled) ancesters.  Each node is identified by a
  // PVNodeID object, which is a physical volume and copy number.  It
  // is a vector of PVNodeIDs corresponding to the geometry hierarchy
  // actually selected, i.e., not culled.
  typedef G4PhysicalVolumeModel::G4PhysicalVolumeNodeID PVNodeID;
  typedef std::vector<PVNodeID> PVPath;
  const PVPath& drawnPVPath = pPVModel->GetDrawnPVPath();
  //G4int currentDepth = pPVModel->GetCurrentDepth();
  //G4VPhysicalVolume* pCurrentPV = pPVModel->GetCurrentPV();
  //G4LogicalVolume* pCurrentLV = pPVModel->GetCurrentLV();
  //G4Material* pCurrentMaterial = pPVModel->GetCurrentMaterial();

  // Actually, it is enough to store the logical volume of current
  // physical volume...
  fDrawnLVStore.insert
    (drawnPVPath.back().GetPhysicalVolume()->GetLogicalVolume());

  // Find mother.  ri points to drawn mother, if any.
  PVPath::const_reverse_iterator ri = ++drawnPVPath.rbegin();
  if (ri != drawnPVPath.rend()) {
    // This volume has a mother.
    G4LogicalVolume* drawnMotherLV =
      ri->GetPhysicalVolume()->GetLogicalVolume();
    if (fDrawnLVStore.find(drawnMotherLV) != fDrawnLVStore.end()) {
      // Mother previously encountered.  Add this volume to
      // appropriate node in scene graph tree.
      // ...
    } else {
      // Mother not previously encountered.  Shouldn't happen, since
      // G4PhysicalVolumeModel sends volumes as it encounters them,
      // i.e., mothers before daughters, in its descent of the
      // geometry tree.  Error!
      G4cerr << "ERROR: G4XXXSceneHandler::PreAddSolid: Mother "
	     << ri->GetPhysicalVolume()->GetName()
	     << ':' << ri->GetCopyNo()
	     << " not previously encountered."
	"\nShouldn't happen!  Please report to visualization coordinator."
	     << G4endl;
      // Continue anyway.  Add to root of scene graph tree.
      // ...
    }
  } else {
    // This volume has no mother.  Must be a top level un-culled
    // volume.  Add to root of scene graph tree.
    // ...
  }
}

void G4VTKSceneHandler::PostAddSolid()
{
    std::cout<<"PostAddSolid Called"<<std::endl;
    rootMBDS->SetBlock(rootMBDS->GetNumberOfBlocks(), leafMBDS);
}
