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
// $Id: G4VTK.cc 85582 2014-10-31 09:07:30Z gcosmo $
//
// 
// Owen Crawford 13th June 2016
// A driver for writing VTK files.
// Modified from XXX template files.

#include "G4VTK.hh"
#include "G4VTKSceneHandler.hh"
#include "G4VTKViewer.hh"

#include <stdio>

G4VTK::G4VTK():
  G4VGraphicsSystem("G4VTK",
		    "VTK",
		    "Driver that outputs VTK",
		    G4VGraphicsSystem::fileWriter),
  sceneHandler(nullptr),
  viewer(nullptr)
{
    std::cout << "I exist!" << std::endl;
  //TODO G4VTKMessenger::GetInstance() goes here.
}

G4VTK::~G4VTK() {}

G4VSceneHandler* G4VTK::CreateSceneHandler(const G4String& name) {
  if (sceneHandler) {
    std::cout << "G4VTK::CreateSceneHandler: Cannot create more than one G4VTKSceneHandler" << std::endl;
    return NULL;
  }
  sceneHandler = new G4VTKSceneHandler(*this, name);
  return sceneHandler;
}

G4VViewer* G4VTK::CreateViewer (G4VSceneHandler& scene, const G4String& name) {
  if (viewer != NULL) {
    cout << "G4VTK::CreateViewer: Cannot create more than one G4VTKViewer" << endl;
    return NULL;
  }
  viewer  = new G4VTKViewer ((G4VTKSceneHandler&)scene, name);
  return viewer;
}

void G4VTK::removeSceneHandler() {
  // actual deletion is done in VisManager
  sceneHandler = nullptr;
}

void G4VTK::removeViewer() {
  // actual deletion is done in VisManager
  viewer = nullptr;
}


