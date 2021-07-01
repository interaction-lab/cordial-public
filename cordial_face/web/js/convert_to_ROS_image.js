facePublisher = new ROSLIB.Topic({
    ros : ros,
    name : '/face/js',
    messageType : 'sensor_msgs/CompressedImage',
    });

window.setInterval(sendSVGMessage, 50)


function sendSVGMessage() {
    console.time('send message')
    //convert svg to string to draw on a canvas
    var svgURL = new XMLSerializer().serializeToString(document.querySelector('svg'));
    var img = new Image();

    //once the url loads, draw to the canvas
    img.onload = function() {
        invisible_ctxt.drawImage(this, 0, 0);

        //convert the data to an image URL
        var data = invisible_canvas.toDataURL('image/jpeg');

        //send a compressed image
        var imageMessage = new ROSLIB.Message({
            format : "jpeg",
            data : data.replace("data:image/jpeg;base64,", "")
        })
        facePublisher.publish(imageMessage);
        console.timeEnd('send message')
    }
    
    //set the URL so the magic can happen!
    img.src = 'data:image/svg+xml; charset=utf8, ' + encodeURIComponent(svgURL);
    
  }


// set up canvas object that we use to draw the svg
var invisible_canvas = document.createElement('canvas');
//set height and width to match what the face is rendered on
invisible_canvas.width = window.innerWidth
invisible_canvas.height = window.innerHeight

var invisible_ctxt = invisible_canvas.getContext('2d');
    