// //general rule of thumb: any code that relies on data from the environment should be non-blocking. i.e.: moving is always non-blocking because you need to loop in order to line sense

// void burger() {
//   //client.send("burger routine starting")
//   if (currentStation() != "BUNS") {//fix with strcmp
//     goToStation("BUNS"); //non-blocking
//   } else {
//     pickUp(); //could be either, probably better if blocking
//     if (hasObject() == "bun") {//fix
//       turn180(); //blocking
//     } else {return /*??????*/}
//     if (currentStation != "PLATES") {
//       goToStation("PLATES");
//     } else {
//       putDown();
//       if (hasObject() == "false") {
//         turn180();
//       } else {return}

//     }
    
//   }
// }

// void burger() {
//   switch (stage) {
//     case "going buns":
//       if (currentStation() != "BUNS") {//fix with strcmp
//         goToStation("BUNS"); //non-blocking
//       } else {
//         stage = "grabbing";
//       }
//     case "grabbing":
//       pickUp(); //blocking
//       if (hasObject()) { //this if may not be needed (i.e. assume you have picked up the object)
//         turn180(); //blocking
//       } else {return /*??????*/}
//   }
// }