#pragma once
//status for ASCOM communications, returns as integer 0-4
enum Shutterstate { shutterOpen, shutterClosed, shutterOpening, shutterClosing, shutterError }; // as per ASCOM definition
enum Roofcommands { open, close, stop };  // for movecommand
Shutterstate DynamicRoof(Shutterstate shutter);
Shutterstate MoveRoof(Roofcommands command, Shutterstate shutter);
