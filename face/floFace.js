
//The Little FLO simulation program 


//Set all values in terms of inches
var perimX = 5.3;
var perimY = 4.8;
var eyeSide = 0.75;
var eyeBetween = .095;
var eyeDiam = .085;
var mouthHorz = 2.5;
var mouthVert = mouthHorz / 2;
var mouthBetween = 0.155;
var mouthDiam = 0.15;
var mag = 150;
var svgns = "http://www.w3.org/2000/svg";

//Using magnification, approximately convert inches to pixels in html
var perimXMag = perimX * mag;
var perimYMag = perimY * mag;
var eyeSideMag = eyeSide * mag;
var eyeBetweenMag = eyeBetween * mag;
var eyeDiamMag = eyeDiam * mag;
var mouthHorzMag = mouthHorz * mag;
var mouthVertMag = mouthHorzMag / 2;
var mouthBetweenMag = mouthBetween * mag;
var mouthDiamMag = mouthDiam * mag;

//Declare location variables, in terms of pixels
var leftEyeLocX = 650;
var leftEyeLocY = 250;
var rightEyeLocX = 1035;
var rightEyeLocY = 250;
var mouthLocX = 715;
var mouthLocY = 500;

var leftCount = new Array();

for (i = 0; i < 8; i++) {
    leftCount[i] = new Array();
    for (j = 0; j < 8; j++) {
        leftCount[i][j] = false;
    }
}

function leftSwap(i, j) {
    var attribute =  document.getElementById("innerLeft[" + i + "," + j +"]").getAttribute("fill")
    console.log(attribute)
   if (attribute === "blue") {
       document.getElementById("innerLeft[" + i + "," + j +"]").setAttribute("fill", "gray")
   }
else if(attribute === "gray") {
    document.getElementById("innerLeft[" + i + "," + j +"]").setAttribute("fill", "blue")
}
else {
    console.log("Oops")
}
}

function rightSwap(i, j) {
    var attribute =  document.getElementById("innerRight[" + i + "," + j +"]").getAttribute("fill")
    console.log(attribute)
   if (attribute === "blue") {
       document.getElementById("innerRight[" + i + "," + j +"]").setAttribute("fill", "gray")
   }
else if(attribute === "gray") {
    document.getElementById("innerRight[" + i + "," + j +"]").setAttribute("fill", "blue")
}
else {
    console.log("Oops")
}
}

function mouthSwap(i, j) {
    var attribute =  document.getElementById("innerMouth[" + i + "," + j +"]").getAttribute("fill")
    console.log(attribute)
   if (attribute === "blue") {
       document.getElementById("innerMouth[" + i + "," + j +"]").setAttribute("fill", "gray")
   }
else if(attribute === "gray") {
    document.getElementById("innerMouth[" + i + "," + j +"]").setAttribute("fill", "blue")
}
else {
    console.log("Oops")
}
}

//Paramaterize Flo's dimensions
function setValues() {
    var init = false;
    document.getElementById("facePerim").setAttribute("rx", perimXMag / 2);
    document.getElementById("facePerim").setAttribute("ry", perimYMag / 2);
    document.getElementById("leftEye").setAttribute("height", eyeSideMag);
    document.getElementById("leftEye").setAttribute("width", eyeSideMag);
    document.getElementById("leftEye").setAttribute("x", leftEyeLocX);
    document.getElementById("leftEye").setAttribute("y", leftEyeLocY);
    document.getElementById("rightEye").setAttribute("height", eyeSideMag);
    document.getElementById("rightEye").setAttribute("width", eyeSideMag);
    document.getElementById("mouth").setAttribute("width", mouthHorzMag);
    document.getElementById("rightEye").setAttribute("x", rightEyeLocX);
    document.getElementById("rightEye").setAttribute("y", rightEyeLocY);
    document.getElementById("mouth").setAttribute("height", mouthVertMag);
    document.getElementById("mouth").setAttribute("x", mouthLocX);
    document.getElementById("mouth").setAttribute("y", mouthLocY);
    for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            var circle = document.createElementNS(svgns, 'circle')
            circle.setAttributeNS(null, "id", "innerLeft[" + i + "," + j + "]")
            circle.setAttributeNS(null, "cx", (leftEyeLocX + (i + 1 / 2) * eyeBetweenMag));
            circle.setAttributeNS(null, "cy", (leftEyeLocY + (j + 1 / 2) * eyeBetweenMag));
            circle.setAttributeNS(null, "r", (eyeDiamMag / 2));
            circle.setAttributeNS(null, "fill", "gray");
            circle.setAttributeNS(null, "width", "50");
            circle.setAttributeNS(null, "height", "50");
            circle.setAttributeNS(null, "stroke", "black");
            circle.setAttributeNS(null, "stroke-width", 1);
            circle.setAttributeNS(null, "onclick", "leftSwap(" + i + "," + j + ")");
            document.getElementById('full').appendChild(circle);
        }
    }
      for (i = 0; i < 8; i++) {
        for (j = 0; j < 8; j++) {
            var circle = document.createElementNS(svgns, 'circle')
            circle.setAttributeNS(null, "id", "innerRight[" + i + "," + j + "]")
            circle.setAttributeNS(null, "cx", (rightEyeLocX + (i + 1 / 2) * eyeBetweenMag));
            circle.setAttributeNS(null, "cy", (rightEyeLocY + (j + 1 / 2) * eyeBetweenMag));
            circle.setAttributeNS(null, "r", (eyeDiamMag / 2));
            circle.setAttributeNS(null, "fill", "gray");
            circle.setAttributeNS(null, "width", "50");
            circle.setAttributeNS(null, "height", "50");
            circle.setAttributeNS(null, "stroke", "black");
            circle.setAttributeNS(null, "stroke-width", 1);
            circle.setAttributeNS(null, "onclick", "rightSwap(" + i + "," + j + ")");
            document.getElementById('full').appendChild(circle);
        }
    }
          for (i = 0; i < 16; i++) {
        for (j = 0; j < 8; j++) {
            var circle = document.createElementNS(svgns, 'circle')
            circle.setAttributeNS(null, "id", "innerMouth[" + i + "," + j + "]")
            circle.setAttributeNS(null, "cx", (mouthLocX + (i + 1 / 2) * mouthBetweenMag));
            circle.setAttributeNS(null, "cy", (mouthLocY + (j + 1 / 2) * mouthBetweenMag));
            circle.setAttributeNS(null, "r", (mouthDiamMag / 2));
            circle.setAttributeNS(null, "fill", "gray");
            circle.setAttributeNS(null, "width", "50");
            circle.setAttributeNS(null, "height", "50");
            circle.setAttributeNS(null, "stroke", "black");
            circle.setAttributeNS(null, "stroke-width", 1);
            circle.setAttributeNS(null, "onclick", "mouthSwap(" + i + "," + j + ")");
            document.getElementById('full').appendChild(circle);
        }
    }
    }

