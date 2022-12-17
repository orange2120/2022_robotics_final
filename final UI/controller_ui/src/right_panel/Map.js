
//Map of the agv
export default function Map({ip}) {
    return (
      <div  >
        <img style={{width:200,stroke:"white"}} src={ip+":5000/video"} alt="Video"/>
      </div>
    );
  }