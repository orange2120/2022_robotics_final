import Move from "./Move_control";
import Speedometer from "./SpeedMeter";
import Camera from "./Map";
import Fire from "./Fire";
import { Stack} from "@mui/system";

export default function RightPanel({data,socket,ip}) {
    return (
      <Stack sx={{m:1}}  direction={"column"} alignItems={"center"}>
        <Camera ip={ip}/>
        <Speedometer velocity={data.velocity}/>
        <Stack sx={{m:-4}} direction={"row"} alignItems={"center"} spacing={2}>
          <Move socket={socket}/>
          <Fire socket={socket} />
        </Stack>
      </Stack>
    );
  }