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
// $Id: G4VTKViewer.cc 66373 2012-12-18 09:41:34Z gcosmo $
//
// 
// Owen Crawford 13th June 2016
// A driver for writing VTK files.
// Modified from XXX template files.

#include "G4VTKViewer.hh"
#include "G4VTK.hh"

#include "G4VSceneHandler.hh"
#include "G4VTKSceneHandler.hh"
#include <sstream>

#include "vtkXMLMultiBlockDataWriter.h"

void G4VTKViewer::FileWriter::Close()
{
  if (fOpen) {
    G4cout << "Closing file " << fFileName << G4endl;
    //fFile.close();
    fOpen = false;
  }
}

void G4VTKViewer::FileWriter::WriteItem(const vtkSmartPointer<vtkMultiBlockDataSet> item)
{
  std::cout<<"WriteItem called"<<std::endl;
  if (!fOpen)
    {
      //std::cout<<"Opening file..."<<std::endl;
      mbdw = vtkSmartPointer<vtkXMLMultiBlockDataWriter>::New();
      
      std::ifstream ifs;
      std::ostringstream oss;
      G4int i = 0;
      
      do {
	oss.str("");
	oss << "g4_" << i << "." << mbdw->GetDefaultFileExtension();
	ifs.open(oss.str().c_str());
	if (!ifs) break;  // Doesn't exist, so can open a new file.
	else ifs.close();
	++i;
      } while(true);
      fFileName = oss.str();
      G4cout << "Opening file " << fFileName << G4endl;
      mbdw->SetFileName(fFileName.c_str());
      //fFile.open(fFileName.c_str());
      fOpen = true;
    }
    mbdw->SetInputData(item);
//    mbdw->Print(std::cout);
    mbdw->Write();
  
//  if (fFile.good()) fFile << item << std::endl;
//  else G4cout << "G4VTKViewer::FileWriter::WriteItem: ERROR" << G4endl;
}

G4VTKViewer::G4VTKViewer
(G4VSceneHandler& sceneHandler, const G4String& name):
  G4VViewer(sceneHandler, sceneHandler.IncrementViewCount(), name),
  geometryIncluded(false)
{}

G4VTKViewer::~G4VTKViewer()
{
  G4VTK* pVTKSystem = dynamic_cast<G4VTK*>(GetSceneHandler()->GetGraphicsSystem());
  if (pVTKSystem) pVTKSystem->removeViewer();
  fFileWriter.Close();
}

void G4VTKViewer::SetView() {
#ifdef G4VTKDEBUG
  G4cout << "G4VTKViewer::SetView() called." << G4endl;
#endif
}

void G4VTKViewer::ClearView() {
#ifdef G4VTKDEBUG
  G4cout << "G4VTKViewer::ClearView() called." << G4endl;
#endif
  //fFileWriter.Rewind();
}

void G4VTKViewer::DrawView() {
    
    std::cout<<"DrawView called"<<std::endl;
    
#ifdef G4VTKDEBUG
  G4cout << "G4VTKViewer::DrawView() called." << G4endl;
#endif

  if ( !geometryIncluded ) {
    // Draw the geometry
    NeedKernelVisit();
    ProcessView();
    geometryIncluded = true;
  }

}

//void G4VTKViewer::ProcessView(){
    
//}

void G4VTKViewer::ShowView() {
#ifdef G4VTKDEBUG
  G4cout << "G4VTKViewer::ShowView() called." << G4endl;
#endif
  G4VViewer::ShowView();

  G4VTKSceneHandler* sceneHandler = dynamic_cast<G4VTKSceneHandler*>(GetSceneHandler());
  if ( sceneHandler ) {
    if ( sceneHandler->closeVTK() ) {
      sceneHandler->openVTK();

      //G4HepRepMessenger* messenger = G4HepRepMessenger::GetInstance();
      //if (messenger->appendGeometry()) geometryIncluded = false;
    }
  }
}

void G4HepRepViewer::FinishView () {
#ifdef SDEBUG
  cout << "G4HepRepViewer::FinishView" << endl;
#endif
  G4VViewer::FinishView();
}

void G4HepRepViewer::reset() {
  geometryIncluded = false;
}
