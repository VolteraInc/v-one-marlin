#include "../../libraries/PlacementNew/PlacementNew.h"

// define a ToolCarriage class so we can hide this stuff in it
union {
  tools::NoTool noTool;
  tools::Probe probe;
  tools::Dispenser dispenser;
  tools::Router router;
} toolBuffer;

inline void emplace(Tool* toolBuffer, ToolType type) {
  if (toolBuffer) {
    toolBuffer.~Tool();
  }
  switch (type) {
    case ToolType::Probe: new (toolBuffer) Probe() break;
  }
}

// TODO use a Union to pre-allocate space for the tool then use placement-new/delete
//   - the union is just to get the size right, not for accessing the object
//     though we could do that if virtual calls end up horrifyingly bad
//   - otherwise we'll need pre-allocate all of them and add init() de-init() to the classes
//
// currentTool->isr(); // i like this idea...feels good to allow router to confirm/detect detachs whenever/however it likes
//   if (currentTool->detached()) {
//     if (!stepper.empty()) {
//       error
//     }
//     stepper.quickStop();
//
//   }
// later (in slowPath ?)
// if (currentTool->detached()) {
//   setTool(ToolType::AttachmentDetector);
// }
