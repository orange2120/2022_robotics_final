import React from 'react'
import {ReactComponent as FireIcon} from './images/fire.svg';
import { useState } from 'react';
import { IconButton } from '@mui/material';


export default function Fire({socket}) {
  const styleUnClick={
    transform: `rotate(-40deg)`,
    fill:'white'

  }
  const styleOnEnter={
    transform: `rotate(-40deg)`,
    stroke:"red",
    fill:'orange',
    outerHeight:300,
  }

  const onClickHandle=()=>{
    console.log();
    
    socket.emit("fire_topic/ui_to_agv",true);
  }
  const [style,setStyle] = useState(styleUnClick)
  
    return (
      // <IconButton onClick={onClickHandle}  onMouseEnter={()=>setStyle(styleOnEnter)} onMouseLeave={()=>setStyle(styleUnClick)}>
      //   {/* <svg style={{width:120, transform: `rotate(-40deg)`, fill:"red",stroke:'green'}} src={FireIcon}/> */}
      //   <FireIcon  style={style}  />
      // </IconButton>
      <IconButton onClick={onClickHandle}  >
      {/* <svg style={{width:120, transform: `rotate(-40deg)`, fill:"red",stroke:'green'}} src={FireIcon}/> */}
      <FireIcon  style={style}  />
    </IconButton>
    );
  }