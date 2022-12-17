import { Box } from "@mui/system";
import { Joystick } from "react-joystick-component";

export default function Move({socket}) {
    return (
      <Box direction={"column"}>
        <Joystick></Joystick>
      </Box>
    );
  }