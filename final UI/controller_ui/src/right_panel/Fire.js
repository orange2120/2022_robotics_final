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
  const [fireFlag,setFireFlag] = useState(false)
  const onClickHandle=()=>{
    if(fireFlag){
      setFireFlag(false);
      socket.emit("fire_topic",false);
    }
    else{
      setFireFlag(true);
      socket.emit("fire_topic",true);
    }
  }
  const [style,setStyle] = useState(styleUnClick)
  
    return (
      // <IconButton onClick={onClickHandle}  onMouseEnter={()=>setStyle(styleOnEnter)} onMouseLeave={()=>setStyle(styleUnClick)}>
      //   {/* <svg style={{width:120, transform: `rotate(-40deg)`, fill:"red",stroke:'green'}} src={FireIcon}/> */}
      //   <FireIcon  style={style}  />
      // </IconButton>
      <IconButton onClick={onClickHandle}   >
      {/* <svg style={{width:120, transform: `rotate(-40deg)`, fill:"red",stroke:'green'}} src={FireIcon}/> */}
      <FireIcon  style={style}  />
    </IconButton>
    );
  }