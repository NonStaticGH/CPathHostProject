// Copyright Dominik Trautman. Published in 2022. All Rights Reserved.

#include "CPathfinding.h"

#define LOCTEXT_NAMESPACE "FCPathfindingModule"

void FCPathfindingModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
}

void FCPathfindingModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FCPathfindingModule, CPathfinding)