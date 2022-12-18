import { Box } from "@mui/system";
import { Joystick } from "react-joystick-component";
import { useState } from "react";


function GobeJoystickController({
  move,
  start,
  stop,
}) {
  
  
  return (
    <div >
        <Joystick
          move={move}
          stop={stop}
          start={start}
        />
    </div>
  );
}
export default function Move({socket}) {
  const handleMove = (e) => {
    socket.emit("move_topic",e)
  };
  const handleStop = (e) => {
    console.log(e);
    socket.emit("move_topic",e);
  };
  const handleStart = (e) => {
    console.log(e);
    
  };
    return (
      <Box direction={"column"}>
        <GobeJoystickController
          move={handleMove}
          stop={handleStop}
          start={handleStart}
        />
      </Box>
    );
  }