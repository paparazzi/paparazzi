// EShapes.js
//
// Based on an idea, and some lines of code, by "thetoy" 
//
//   This Javascript is provided by Mike Williams
//   Community Church Javascript Team
//   http://www.bisphamchurch.org.uk/   
//   http://econym.org.uk/gmap/
//
//   This work is licenced under a Creative Commons Licence
//   http://creativecommons.org/licenses/by/2.0/uk/
//
// Version 0.0 04/Apr/2008 Not quite finished yet
// Version 1.0 10/Apr/2008 Initial release

      GPolyline.Shape = function(point,r1,r2,r3,r4,rotation,vertexCount,  colour,weight,opacity,opts,tilt) {
        var rot = -rotation*Math.PI/180;
        var points = [];
	var latConv = point.distanceFrom(new GLatLng(point.lat()+0.1,point.lng()))*10;
 	var lngConv = point.distanceFrom(new GLatLng(point.lat(),point.lng()+0.1))*10;
 	var step = (360/vertexCount)||10;
        
        var flop = -1;
        if (tilt) {
          var I1=180/vertexCount;
        } else {
          var  I1=0;
        }
        for(var i=I1; i<=360.001+I1; i+=step) {
          var r1a = flop?r1:r3;
          var r2a = flop?r2:r4;
          flop = -1-flop;
 	  var y = r1a * Math.cos(i * Math.PI/180);
 	  var x = r2a * Math.sin(i * Math.PI/180);
          var lng = (x*Math.cos(rot)-y*Math.sin(rot))/lngConv;
          var lat = (y*Math.cos(rot)+x*Math.sin(rot))/latConv;

 	  points.push(new GLatLng(point.lat()+lat,point.lng()+lng));
        }
	return (new GPolyline(points,colour,weight,opacity,opts))
      }

      GPolyline.Circle = function(point,radius,colour,weight,opacity,opts) {
        return GPolyline.Shape(point,radius,radius,radius,radius,0,100,colour,weight,opacity,opts)
      }

      GPolyline.RegularPoly = function(point,radius,vertexCount,rotation,colour,weight,opacity,opts) {
        rotation = rotation||0;
        var tilt = !(vertexCount&1);
        return GPolyline.Shape(point,radius,radius,radius,radius,rotation,vertexCount,colour,weight,opacity,opts,tilt)
      }

      GPolyline.Star = function(point,r1,r2,points,rotation,colour,weight,opacity,opts) {
        rotation = rotation||0;
        return GPolyline.Shape(point,r1,r1,r2,r2,rotation,points*2,colour,weight,opacity,opts)
      }

      GPolyline.Ellipse = function(point,r1,r2,rotation,colour,weight,opacity,opts) {
        rotation = rotation||0;
        return GPolyline.Shape(point,r1,r2,r1,r2,rotation,100,colour,weight,opacity,opts)
      }




      GPolygon.Shape = function(point,r1,r2,r3,r4,rotation,vertexCount,  strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts,tilt) {
        var rot = -rotation*Math.PI/180;
        var points = [];
	var latConv = point.distanceFrom(new GLatLng(point.lat()+0.1,point.lng()))*10;
 	var lngConv = point.distanceFrom(new GLatLng(point.lat(),point.lng()+0.1))*10;
 	var step = (360/vertexCount)||10;
        
        var flop = -1;
        if (tilt) {
          var I1=180/vertexCount;
        } else {
          var  I1=0;
        }
        for(var i=I1; i<=360.001+I1; i+=step) {
          var r1a = flop?r1:r3;
          var r2a = flop?r2:r4;
          flop = -1-flop;
 	  var y = r1a * Math.cos(i * Math.PI/180);
 	  var x = r2a * Math.sin(i * Math.PI/180);
          var lng = (x*Math.cos(rot)-y*Math.sin(rot))/lngConv;
          var lat = (y*Math.cos(rot)+x*Math.sin(rot))/latConv;

 	  points.push(new GLatLng(point.lat()+lat,point.lng()+lng));
        }
	return (new GPolygon(points,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts))
      }

      GPolygon.Circle = function(point,radius,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts) {
        return GPolygon.Shape(point,radius,radius,radius,radius,0,100,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts)
      }

      GPolygon.RegularPoly = function(point,radius,vertexCount,rotation,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts) {
        rotation = rotation||0;
        var tilt = !(vertexCount&1);
        return GPolygon.Shape(point,radius,radius,radius,radius,rotation,vertexCount,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts,tilt)
      }

      GPolygon.Star = function(point,r1,r2,points,rotation,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts) {
        rotation = rotation||0;
        return GPolygon.Shape(point,r1,r1,r2,r2,rotation,points*2,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts)
      }

      GPolygon.Ellipse = function(point,r1,r2,rotation,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts) {
        rotation = rotation||0;
        return GPolygon.Shape(point,r1,r2,r1,r2,rotation,100,strokeColour,strokeWeight,Strokepacity,fillColour,fillOpacity,opts)
      }




      function EOffset(point,easting,northing) {
	var latConv = point.distanceFrom(new GLatLng(point.lat()+0.1,point.lng()))*10;
 	var lngConv = point.distanceFrom(new GLatLng(point.lat(),point.lng()+0.1))*10;
        return new GLatLng(point.lat()+northing/latConv,point.lng()+easting/lngConv)      
      }

      function EOffsetBearing(point,dist,bearing) {
	var latConv = point.distanceFrom(new GLatLng(point.lat()+0.1,point.lng()))*10;
 	var lngConv = point.distanceFrom(new GLatLng(point.lat(),point.lng()+0.1))*10;
        var lat=dist * Math.cos(bearing * Math.PI/180)/latConv;
        var lng=dist * Math.sin(bearing * Math.PI/180)/lngConv; 
        return new GLatLng(point.lat()+lat,point.lng()+lng)      
      }


