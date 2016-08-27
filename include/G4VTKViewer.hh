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
// $Id: G4VTKViewer.hh 66373 2012-12-18 09:41:34Z gcosmo $
//
// 
// Owen Crawford 13th June 2016
// A driver for writing VTK files.
// Modified from XXX template files.

#ifndef G4VTKVIEWER_HH
#define G4VTKVIEWER_HH

#include "G4VViewer.hh"

#include <fstream>

#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkSmartPointer.h>

class G4VTKViewer: public G4VViewer {
public:
  G4VTKViewer(G4VSceneHandler&,const G4String& name = "");
  virtual ~G4VTKViewer();
  void SetView() override ;
  void ClearView() override ;
  void DrawView() override;
  
  //void ProcessView();
  
  void ShowView() override;

  // TODO: Missing are FinishView (override) and reset

private:
  bool geometryIncluded;

public:

  // A simple class to handle delayed opening, etc.  There are various
  // degrees of sophistication in, for example, the allocation of
  // filenames -- see FukuiRenderer or HepRepFile.
  class FileWriter {
  public:
    FileWriter(): fOpen(false) {}
    G4bool IsOpen() {return fOpen;}
    void WriteItem(const vtkSmartPointer<vtkMultiBlockDataSet> item);
    void Close();
    // Implement rewind as close and re-open...
    //void Rewind() {if (fOpen) {fFile.close(); fFile.open(fFileName.c_str());}}
  private:
    vtkSmartPointer<vtkXMLMultiBlockDataWriter> mbdw;
    G4String fFileName;
    G4bool fOpen;
    //std::ofstream fFile;
    
  };
  FileWriter& GetFileWriter() {return fFileWriter;}
private:

    FileWriter fFileWriter;
};

#endif
