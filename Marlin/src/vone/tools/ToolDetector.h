#pragma once

class PTopPin;

class ToolDetector {

  public:

    ToolDetector(PTopPin& pin);

    void update();

    bool enabled() { return _enabled; }
    void enable(bool enable) { _enabled = enable; }

    ToolType toolType();
    // setTool(ToolType type, bool force);

  private:
    bool _enabled = true;
    PTopPin& _ptopPin;
};

// TODO: could have tool detector see tool changes,
// but apply the detected tool type to the vone at the end of the processing loop
// This would reduce (maybe eliminate) the need to remember deteches inside PTopPin


// - if tooldetector is reading from ptop to detect changes we _could_ miss a change
// - change vs detact?
// - if drill can signal reset for 500ms


// - detecting a detach is a safety issue, so an fast (isr-based) detector make sense
//     - could be tool specific (Router has detachDetector) or general
//       specific would mean an isr for each tool...not unreasonable
//        - the NoneTool would become a ToolAttachmentDetector
//    - on detach
//        - stepper.quickStop()
//        - set tool to NoneTool
