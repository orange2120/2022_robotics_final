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
  
  socket.on("mode_test",(data)=>{
    console.log("hi");
    socket.broadcast.emit("mode_topic/ui_to_agv","hi");
  });
  socket.on("mode_test",(data)=>{
    socket.broadcast.emit("target_point/ui_to_agv",data);
  });
  
  

  socket.on("disconnect", () => {
    console.log("User Disconnected", socket.id);
  });
});

server.listen(3001,() => {
//server.listen(3001,"192.168.72.22",() => {
  console.log("SERVER RUNNING");
});


