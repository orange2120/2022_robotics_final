import {Stack} from "@mui/system";
import { Battery0Bar,Battery2Bar ,Battery4Bar ,Battery6Bar ,BatteryFull  } from "@mui/icons-material";


export default function Battery({number}) {
    if(number>=80){
        return (
            <Stack direction="row" alignItems="center">
              <BatteryFull sx={{color:"green",fontSize: 70, transform: "rotate(90deg)",ml:1}}/>
              <h2 style={{color:"white"}}>{number}%</h2>
            </Stack>
          );
    }
    else if (number>=60) {
        return (
            <Stack direction="row" alignItems="center">
              <Battery6Bar sx={{color:"white",fontSize: 70, transform: "rotate(90deg)",ml:1}}/>
              <h2 style={{color:"white"}}>{number}%</h2>
            </Stack>
          );
    }

    else if (number>=40) {
        return (
            <Stack direction="row" alignItems="center">
              <Battery4Bar sx={{color:"yellow",fontSize: 70, transform: "rotate(90deg)",ml:1}}/>
              <h2 style={{color:"white"}}>{number}%</h2>
            </Stack>
          );
    }
    else if (number>=20){
        return (
            <Stack direction="row" alignItems="center">
              <Battery2Bar sx={{color:"orange",fontSize: 70, transform: "rotate(90deg)",ml:1}}/>
              <h2 style={{color:"white"}}>{number}%</h2>
            </Stack>
          );
    }
    else {
        return (
            <Stack direction="row" alignItems="center">
              <Battery0Bar sx={{color:"red",fontSize: 70, transform: "rotate(90deg)",ml:1}}/>
              <h2 style={{color:"white"}}>{number}%</h2>
            </Stack>
          );
    }
    
    
  }