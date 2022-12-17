import React from 'react'
import {ReactComponent as BulletIcon} from './images/bullets.svg';
import { Stack } from '@mui/system';


export default function Bullets({number}) {
  
  
    return (
      <Stack direction="row" alignItems="center" sx={{m:1}}>
        <svg style={{width:60,height:60}}>
            <BulletIcon fill='white' />
        </svg>
        <h2 style={{color:"white", width:"60px"}}>{number}</h2>
      </Stack>
    );
  }