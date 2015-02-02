'use strict';

/* Controllers */

angular.module('pprzmon.controllers', [])
  .controller('MainController', ['$scope', '$log', '$http', 'socket', 'uiGmapGoogleMapApi', function($scope, $log, $http, socket, GoogleMapApi ) {

    google.maps.visualRefresh = true;

	// on connection to server, ask for ac_id to monitor
	socket.on('connect', function(){
		// call the server-side function 'subscribe' and send one parameter (value of prompt)
		socket.emit('subscribe', prompt("Filter on which aircraft?"));
	});

	// listener, whenever the server emits a message, this updates the messages list
	socket.on('FLIGHT_PARAM', function(data) {
        var elems = data.split(" ");
        var lat = parseFloat( elems[ 4 ] );
        var lon = parseFloat( elems[ 5 ] );

        $log.log( lat );
        $log.log( lon );

        var sw = $scope.map.bounds.getSouthWest();
        var ne = $scope.map.bounds.getNorthEast();
        if (( lat < sw.lat() ) || ( lat > ne.lat() ) || ( lon < sw.lng() ) || ( lon > ne.lng() ) ) {
            $scope.map.center.latitude = lat;
            $scope.map.center.longitude = lon;
        }

        $scope.uavMarker.coords.latitude = lat;
        $scope.uavMarker.coords.longitude = lon;
	});

	socket.on('ENGINE_STATUS', function(data) {

	});

	socket.on('AP_STATUS', function(data) {

	});

	socket.on('TELEMETRY_STATUS', function(data) {

	});

    $scope.map = {
        center: {
            latitude: -8.062719,
            longitude: -34.8711568
        },
        zoom: 18,
        draggable: true,
        lat: 0.0,
        lng: 0.0,
        panning: false,
        markers: [],
        selectedmarker: null,
        bounds: {},
        windowoptions: {
          boxClass: 'custom-info-window'
        },
        lastEvent: null,
        events: {
            tilesloaded: function (map) {
                $scope.$apply();
            },
            bounds_changed: function( map ) {
                $scope.map.bounds = map.getBounds();
                $scope.$apply();
            }
       }
    };

    $scope.uavMarker = {
        id: 0,
        coords: {
            latitude: 40.1451,
            longitude: -99.6680
        },
        icon: 'icons/airplane.png',
        options: { draggable: false },
        events: {
        }
    };

} ] );

