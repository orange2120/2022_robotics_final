import { Stack} from "@mui/system";
import ModeChange from "./ModeChange"
import Battery from "./Battery";
import Bullets from  "./Bullets";
import OrientControl from "./Move_control"

export default function LeftPanel({data,socket}) {
    return (
      <Stack sx={{mt:1,ml:2}}>
        <ModeChange text={"Mode1"} flag={false} socket={socket}/>
        <ModeChange text={"Mode2"} flag={false} socket={socket}/>
        <ModeChange text={"Mode3"} flag={true} socket={socket}/>
        <Battery number={data.battery}/>
        <Stack direction={"row"} sx={{m:1}} spacing={1}>
            <Bullets number={data.bullets}/>
            <OrientControl socket={socket}/>
        </Stack>
        
      </Stack>
    );
  }