import ReactSpeedometer from "react-d3-speedometer";
import { Box } from "@mui/system";

export default function Speedometer({velocity}) {
  return (
    <Box sx={{m:1}}>
      <ReactSpeedometer
        width={210}
        height={160}
        maxValue={500}
        value={velocity}
        needleColor="white"
        startColor="
        rgba(87, 146, 159, 1)"
        segments={5}
        endColor="rgba(20, 53, 59, 1)"
      />
      </Box>)
}