import { Stack } from "@mui/system";
import { useEffect,useState } from "react";

export default function Camera({ip,socket}) {
  const image_origin = {x:391,y:8};
  const image_end_point = {x:1031,y:488}
  const [mousePos,setMousePos] = useState({})
  useEffect(() => {
    const handleMouseMove = (event) => {
      setMousePos({ x: event.clientX-image_origin.x, y: event.clientY-image_origin.y });
    };
    window.addEventListener('mousemove', handleMouseMove);
    return () => {
      window.removeEventListener(
        'mousemove',
        handleMouseMove
      );
    };
  }, []);
  const handleClick=()=>{
    console.log(mousePos);
    socket.emit("target_point/ui_to_agv");
  }
  return (
    <Stack sx={{mt:1,mb:1}}>
      <img onClick={handleClick}  src={ip+":5000/video"} alt="Video"/>
      {/* <p style={{color:"white"}}>{mousePos.x} {mousePos.y}</p> */}
    </Stack>
  );
}