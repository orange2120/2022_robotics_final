import Camera from "./Camera";
import RightPanel from "./right_panel";
import LeftPanel from "./left_panel";
import { Stack, Box} from "@mui/system";
import { useState, useEffect} from "react";
import { io } from "socket.io-client";
var ip = "http://localhost"
const socket = io.connect(ip);

export default function App() {
  const [data, setData] = useState({
    bullets:100,
    battery:100,
    velocity:100
});
useEffect(() => {
  fetch(ip+":5000/flaskapi").then((res) =>
      res.json().then((data) => {
          setData({
            bullets:data.bullets,
            battery:data.battery,
            velocity:data.velocity
          });
      })
  );
  console.log(data)
}, []);
  return (
    <Stack direction="row" justifyContent="center" >
      <Box
      sx={{
        width: 300,
        backgroundColor: 'black',
      }}>
        <LeftPanel data={data} socket={socket}/>
      </Box>
      <Box sx={{
        backgroundColor: 'black'
      }}><Camera ip={ip} socket={socket} style={{width:"200px"}}/></Box>
      <Box
      sx={{
        width: 300,
        backgroundColor: 'black',
      }}
    ><RightPanel data={data} ip={ip} socket={socket}/></Box>
    </Stack>
  );
}
