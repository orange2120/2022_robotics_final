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
  const joystick_threshold = 1;
  const [velocity,setVelocity] = useState({x:0,y:0})
  const handleMove = (e) => {
    //console.log(e);
    if(velocity.x>velocity.y & velocity.y<joystick_threshold) setVelocity({x:e.x*e.distance,y:0});
    else if(velocity.y>velocity.x & velocity.x<joystick_threshold) setVelocity({x:0,y:e.y*e.distance});
    else setVelocity({x:e.x*e.distance,y:e.y*e.distance});
    console.log(velocity);
  };
  const handleStop = (e) => {
    //console.log(e);
    setVelocity({x:0,y:0});
    console.log(velocity);
  };
  const handleStart = (e) => {
    //console.log(e);
    setVelocity({x:e.x*e.distance,y:e.y*e.distance});
    console.log(velocity);
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