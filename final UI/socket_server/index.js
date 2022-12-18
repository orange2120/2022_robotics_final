const express = require("express");
const app = express();
const http = require("http");
const cors = require("cors"); 
const { Server } = require("socket.io");
app.use(cors());

const server = http.createServer(app);

const io = new Server(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"],
  },
});

io.on("connection", (socket) => {
  console.log(`User Connected: ${socket.id}`);
  
  socket.on("join_app", (data) => {
    
  });
  
  socket.on("fire_topic",(data)=>{
    console.log(data);
    socket.broadcast.emit("fire_topic/ui_to_agv",1200);
  });
  socket.on("move_topic",(data)=>{
    console.log(data)
    let val = 0;
    if(data.direction=="FORWARD") {
      val = 309+data.distance*(1005-309)/100;
      socket.broadcast.emit("move_topic/ui_to_agv",val);}
    else if(data.direction=="BACKWARD") {
      val = 1005+data.distance*(1685-1005)/100;
      socket.broadcast.emit("move_topic/ui_to_agv",val);}
    else if(data.direction=="RIGHT") {
      val = 945+data.distance*(1666-945)/100;
      socket.broadcast.emit("turn_topic/ui_to_agv",val);}
    else if(data.direction=="LEFT") {
      val = 309+data.distance*(945-309)/100;
      socket.broadcast.emit("turn_topic/ui_to_agv",val);}
    else {
      val = 1000;
      socket.broadcast.emit("move_topic/ui_to_agv",1000);
      socket.broadcast.emit("turn_topic/ui_to_agv",984);
    }
    console.log(val);
    //socket.broadcast.emit("move_topic/ui_to_agv",data);
  });
  
  socket.on("orient_topic",(data)=>{
    let val = 0;
    if(data.direction=="FORWARD") {
      val = 75+data.distance*(1925-75)/100;
      socket.broadcast.emit("orient_topic/ui_to_agv",val);}
    else if(data.direction=="BACKWARD") {
      val = 75+data.distance*(1925-75)/100;
      socket.broadcast.emit("orient_topic/ui_to_agv",val);}
    else if(data.direction=="RIGHT") {
      val = 1020+data.distance*(1624-1020)/100;
      socket.broadcast.emit("shift_topic/ui_to_agv",val);}
    else if(data.direction=="LEFT") {
      val = 309+data.distance*(1020-309)/100;
      socket.broadcast.emit("shift_topic/ui_to_agv",val);}
    else {
      val = 1000;
      socket.broadcast.emit("shift_topic/ui_to_agv",1000);
    }
    console.log(val);
    // socket.broadcast.emit("orient_topic/ui_to_agv",data);
  });
  
  

  socket.on("disconnect", () => {
    console.log("User Disconnected", socket.id);
  });
});

server.listen(3001,() => {
//server.listen(3001,"192.168.72.22",() => {
  console.log("SERVER RUNNING");
});


