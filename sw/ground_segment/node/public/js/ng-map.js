var ngMap = angular.module('ngMap', []);

/**
 * @ngdoc service
 * @name Attr2Options
 * @description 
 *   Converts tag attributes to options used by google api v3 objects, map, marker, polygon, circle, etc.
 */
/*jshint -W030*/
ngMap.service('Attr2Options', ['$parse', 'NavigatorGeolocation', 'GeoCoder', function($parse, NavigatorGeolocation, GeoCoder) { 
  var SPECIAL_CHARS_REGEXP = /([\:\-\_]+(.))/g;
  var MOZ_HACK_REGEXP = /^moz([A-Z])/;  

  var orgAttributes = function(el) {
    (el.length > 0) && (el = el[0]);
    var orgAttributes = {};
    for (var i=0; i<el.attributes.length; i++) {
      var attr = el.attributes[i];
      orgAttributes[attr.name] = attr.value;
    }
    return orgAttributes;
  }

  var camelCase = function(name) {
    return name.
      replace(SPECIAL_CHARS_REGEXP, function(_, separator, letter, offset) {
        return offset ? letter.toUpperCase() : letter;
      }).
      replace(MOZ_HACK_REGEXP, 'Moz$1');
  }

  var JSONize = function(str) {
    try {       // if parsable already, return as it is
      JSON.parse(str);
      return str;
    } catch(e) { // if not parsable, change little
      return str
        // wrap keys without quote with valid double quote
        .replace(/([\$\w]+)\s*:/g, function(_, $1){return '"'+$1+'":'})
        // replacing single quote wrapped ones to double quote 
        .replace(/'([^']+)'/g, function(_, $1){return '"'+$1+'"'})
    }
  }

  var toOptionValue = function(input, options) {
    var output, key=options.key, scope=options.scope;
    try { // 1. Number?
      var num = Number(input);
      if (isNaN(num)) {
        throw "Not a number";
      } else  {
        output = num;
      }
    } catch(err) { 
      try { // 2.JSON?
        if (input.match(/^[\+\-]?[0-9\.]+,[ ]*\ ?[\+\-]?[0-9\.]+$/)) { // i.e "-1.0, 89.89"
          input = "["+input+"]";
        }
        output = JSON.parse(JSONize(input));
        if (output instanceof Array) {
          var t1stEl = output[0];
          if (t1stEl.constructor == Object) { // [{a:1}] : not lat/lng ones
          } else if (t1stEl.constructor == Array) { // [[1,2],[3,4]] 
            output =  output.map(function(el) {
              return new google.maps.LatLng(el[0], el[1]);
            });
          } else if(!isNaN(parseFloat(t1stEl)) && isFinite(t1stEl)) {
            return new google.maps.LatLng(output[0], output[1]);
          }
        }
      } catch(err2) {
        // 3. Object Expression. i.e. LatLng(80,-49)
        if (input.match(/^[A-Z][a-zA-Z0-9]+\(.*\)$/)) {
          try {
            var exp = "new google.maps."+input;
            output = eval(exp); // TODO, still eval
          } catch(e) {
            output = input;
          } 
        // 4. Object Expression. i.e. MayTypeId.HYBRID 
        } else if (input.match(/^([A-Z][a-zA-Z0-9]+)\.([A-Z]+)$/)) {
          try {
            var matches = input.match(/^([A-Z][a-zA-Z0-9]+)\.([A-Z]+)$/);
            output = google.maps[matches[1]][matches[2]];
          } catch(e) {
            output = input;
          } 
        // 5. Object Expression. i.e. HYBRID 
        } else if (input.match(/^[A-Z]+$/)) {
          try {
            var capitalizedKey = key.charAt(0).toUpperCase() + key.slice(1);
            if (key.match(/temperatureUnit|windSpeedUnit|labelColor/)) {
              capitalizedKey = capitalizedKey.replace(/s$/,"");
              output = google.maps.weather[capitalizedKey][input];
            } else {
              output = google.maps[capitalizedKey][input];
            }
          } catch(e) {
            output = input;
          } 
        } else {
          output = input;
        }
      } // catch(err2)
    } // catch(err)
    return output;
  };

  var setDelayedGeoLocation = function(object, method, param, options) {
    options = options || {};
    var centered = object.centered || options.centered;
    var errorFunc = function() {
      void 0;
      var fallbackLocation = options.fallbackLocation || new google.maps.LatLng(0,0);
      object[method](fallbackLocation);
    };
    if (!param || param.match(/^current/i)) { // sensored position
      NavigatorGeolocation.getCurrentPosition().then(
        function(position) { // success
          var lat = position.coords.latitude;
          var lng = position.coords.longitude;
          var latLng = new google.maps.LatLng(lat,lng);
          object[method](latLng);
          if (centered) {
            object.map.setCenter(latLng);
          }
          options.callback && options.callback.apply(object);
        },
        errorFunc
      );
    } else { //assuming it is address
      GeoCoder.geocode({address: param}).then(
        function(results) { // success
          object[method](results[0].geometry.location);
          if (centered) {
            object.map.setCenter(results[0].geometry.location);
          }
        },
        errorFunc
      );
    }
  };


  var getAttrsToObserve = function(attrs) {
    var attrsToObserve = [];
    if (attrs["ng-repeat"] || attrs.ngRepeat) {  // if element is created by ng-repeat, don't observe any
    } else {
      for (var attrName in attrs) {
        var attrValue = attrs[attrName];
        if (attrValue && attrValue.match(/\{\{.*\}\}/)) { // if attr value is {{..}}
          void 0;
          attrsToObserve.push(camelCase(attrName));
        }
      }
    }
    return attrsToObserve;
  };

  var observeAttrSetObj = function(orgAttrs, attrs, obj) {
    var attrsToObserve = getAttrsToObserve(orgAttrs);
    if (Object.keys(attrsToObserve).length) {
      void 0;
    }
    for (var i=0; i<attrsToObserve.length; i++) {
      observeAndSet(attrs, attrsToObserve[i], obj);
    }
  }

  var observeAndSet = function(attrs, attrName, object) {
    attrs.$observe(attrName, function(val) {
      if (val) {
        void 0;
        var setMethod = camelCase('set-'+attrName);
        var optionValue = toOptionValue(val, {key: attrName});
        void 0;
        if (object[setMethod]) { //if set method does exist
          /* if an location is being observed */
          if (attrName.match(/center|position/) && 
            typeof optionValue == 'string') {
            setDelayedGeoLocation(object, setMethod, optionValue);
          } else {
            object[setMethod](optionValue);
          }
        }
      }
    });
  };

  return {
    /**
     * filters attributes by skipping angularjs methods $.. $$..
     * @memberof Attr2Options
     * @param {Hash} attrs tag attributes
     * @returns {Hash} filterd attributes
     */
    filter: function(attrs) {
      var options = {};
      for(var key in attrs) {
        if (key.match(/^\$/) || key.match(/^ng[A-Z]/)) {
        } else {
          options[key] = attrs[key];
        }
      }
      return options;
    },


    /**
     * converts attributes hash to Google Maps API v3 options  
     * ```
     *  . converts numbers to number   
     *  . converts class-like string to google maps instance   
     *    i.e. `LatLng(1,1)` to `new google.maps.LatLng(1,1)`  
     *  . converts constant-like string to google maps constant    
     *    i.e. `MapTypeId.HYBRID` to `google.maps.MapTypeId.HYBRID`   
     *    i.e. `HYBRID"` to `google.maps.MapTypeId.HYBRID`  
     * ```
     * @memberof Attr2Options
     * @param {Hash} attrs tag attributes
     * @param {scope} scope angularjs scope
     * @returns {Hash} options converted attributess
     */
    getOptions: function(attrs, scope) {
      var options = {};
      for(var key in attrs) {
        if (attrs[key]) {
          if (key.match(/^on[A-Z]/)) { //skip events, i.e. on-click
            continue;
          } else if (key.match(/ControlOptions$/)) { // skip controlOptions
            continue;
          } else {
            options[key] = toOptionValue(attrs[key], {scope:scope, key: key});
          }
        } // if (attrs[key])
      } // for(var key in attrs)
      return options;
    },

    /**
     * converts attributes hash to scope-specific event function 
     * @memberof Attr2Options
     * @param {scope} scope angularjs scope
     * @param {Hash} attrs tag attributes
     * @returns {Hash} events converted events
     */
    getEvents: function(scope, attrs) {
      var events = {};
      var toLowercaseFunc = function($1){
        return "_"+$1.toLowerCase();
      };
      var eventFunc = function(attrValue) {
        var matches = attrValue.match(/([^\(]+)\(([^\)]*)\)/);
        var funcName = matches[1];
        var argsStr = matches[2].replace(/event[ ,]*/,'');  //remove string 'event'
        
        var args = scope.$eval("["+argsStr+"]");
        return function(event) {
          function index(obj,i) {return obj[i]}
          f = funcName.split('.').reduce(index, scope)
          f.apply(this, [event].concat(args));
          scope.$apply();
        }
      }

      for(var key in attrs) {
        if (attrs[key]) {
          if (!key.match(/^on[A-Z]/)) { //skip if not events
            continue;
          }
          
          //get event name as underscored. i.e. zoom_changed
          var eventName = key.replace(/^on/,'');
          eventName = eventName.charAt(0).toLowerCase() + eventName.slice(1);
          eventName = eventName.replace(/([A-Z])/g, toLowercaseFunc);

          var attrValue = attrs[key];
          events[eventName] = new eventFunc(attrValue);
        }
      }
      return events;
    },

    /**
     * control means map controls, i.e streetview, pan, etc, not a general control
     * @memberof Attr2Options
     * @param {Hash} filtered filtered tag attributes
     * @returns {Hash} Google Map options
     */
    getControlOptions: function(filtered) {
      var controlOptions = {};
      if (typeof filtered != 'object')
        return false;

      for (var attr in filtered) {
        if (filtered[attr]) {
          if (!attr.match(/(.*)ControlOptions$/)) { 
            continue; // if not controlOptions, skip it
          }

          //change invalid json to valid one, i.e. {foo:1} to {"foo": 1}
          var orgValue = filtered[attr];
          var newValue = orgValue.replace(/'/g, '"');
          newValue = newValue.replace(/([^"]+)|("[^"]+")/g, function($0, $1, $2) {
            if ($1) {
              return $1.replace(/([a-zA-Z0-9]+?):/g, '"$1":');
            } else {
              return $2; 
            } 
          });
          try {
            var options = JSON.parse(newValue);
            for (var key in options) { //assign the right values
              if (options[key]) {
                var value = options[key];
                if (typeof value === 'string') {
                  value = value.toUpperCase();
                } else if (key === "mapTypeIds") {
                  value = value.map( function(str) {
                    if (str.match(/^[A-Z]+$/)) { // if constant
                      return google.maps.MapTypeId[str.toUpperCase()];
                    } else { // else, custom map-type
                      return str;
                    }
                  });
                } 
                
                if (key === "style") {
                  var str = attr.charAt(0).toUpperCase() + attr.slice(1);
                  var objName = str.replace(/Options$/,'')+"Style";
                  options[key] = google.maps[objName][value];
                } else if (key === "position") {
                  options[key] = google.maps.ControlPosition[value];
                } else {
                  options[key] = value;
                }
              }
            }
            controlOptions[attr] = options;
          } catch (e) {
            void 0;
          }
        }
      } // for

      return controlOptions;
    }, // function

    toOptionValue: toOptionValue,
    camelCase: camelCase,
    setDelayedGeoLocation: setDelayedGeoLocation,
    getAttrsToObserve: getAttrsToObserve,
    observeAndSet: observeAndSet,
    observeAttrSetObj: observeAttrSetObj,
    orgAttributes: orgAttributes

  }; // return
}]); 

/**
 * @ngdoc service
 * @name GeoCoder
 * @description
 *   Provides [defered/promise API](https://docs.angularjs.org/api/ng/service/$q) service for Google Geocoder service
 */
ngMap.service('GeoCoder', ['$q', function($q) {
  return {
    /**
     * @memberof GeoCoder
     * @param {Hash} options https://developers.google.com/maps/documentation/geocoding/#geocoding
     * @example
     * ```
     *   GeoCoder.geocode({address: 'the cn tower'}).then(function(result) {
     *     //... do something with result
     *   });
     * ```
     * @returns {HttpPromise} Future object
     */
    geocode : function(options) {
      var deferred = $q.defer();
      var geocoder = new google.maps.Geocoder();
      geocoder.geocode(options, function (results, status) {
        if (status == google.maps.GeocoderStatus.OK) {
          deferred.resolve(results);
        } else {
          deferred.reject('Geocoder failed due to: '+ status);
        }
      });
      return deferred.promise;
    }
  }
}]);

/**
 * @ngdoc service
 * @name NavigatorGeolocation
 * @description
 *  Provides [defered/promise API](https://docs.angularjs.org/api/ng/service/$q) service for navigator.geolocation methods
 */
ngMap.service('NavigatorGeolocation', ['$q', function($q) {
  return {
    /**
     * @memberof NavigatorGeolocation
     * @param {function} success success callback function
     * @param {function} failure failure callback function
     * @example
     * ```
     *  NavigatorGeolocation.getCurrentPosition()
     *    .then(function(position) {
     *      var lat = position.coords.latitude, lng = position.coords.longitude;
     *      .. do something lat and lng
     *    });
     * ```
     * @returns {HttpPromise} Future object
     */
    getCurrentPosition: function() {
      var deferred = $q.defer();
      if (navigator.geolocation) {
        navigator.geolocation.getCurrentPosition(
          function(position) {
            deferred.resolve(position);
          }, function(evt) {
            void 0;
            deferred.reject(evt);
          }
        );
      } else {
        deferred.reject("Browser Geolocation service failed.");
      }
      return deferred.promise;
    },

    watchPosition: function() {
      return "TODO";
    },

    clearWatch: function() {
      return "TODO";
    }
  };
}]); 


/**
 * @ngdoc service
 * @name StreetView
 * @description
 *  Provides [defered/promise API](https://docs.angularjs.org/api/ng/service/$q) service 
 *  for [Google StreetViewService](https://developers.google.com/maps/documentation/javascript/streetview)
 */
ngMap.service('StreetView', ['$q', function($q) {
  return {
    /**
     * Retrieves panorama id from the given map (and or position)
     * @memberof StreetView
     * @param {map} map Google map instance
     * @param {LatLng} latlng Google LatLng instance  
     *   default: the center of the map
     * @example
     *   StreetView.getPanorama(map).then(function(panoId) {
     *     $scope.panoId = panoId;
     *   });
     * @returns {HttpPromise} Future object
     */
    getPanorama : function(map, latlng) {
      latlng = latlng || map.getCenter();
      var deferred = $q.defer();
      var svs = new google.maps.StreetViewService();
      svs.getPanoramaByLocation( (latlng||map.getCenter), 100, function (data, status) {
        // if streetView available
        if (status === google.maps.StreetViewStatus.OK) {
          deferred.resolve(data.location.pano);
        } else {
          // no street view available in this range, or some error occurred
          deferred.resolve(false);
          //deferred.reject('Geocoder failed due to: '+ status);
        }
      });
      return deferred.promise;
    },
    /**
     * Set panorama view on the given map with the panorama id
     * @memberof StreetView
     * @param {map} map Google map instance
     * @param {String} panoId Panorama id fro getPanorama method
     * @example
     *   StreetView.setPanorama(map, panoId);
     */
    setPanorama : function(map, panoId) {
      var svp = new google.maps.StreetViewPanorama(map.getDiv(), {enableCloseButton: true});
      svp.setPano(panoId);
    }
  }; // return
}]);

/**
 * @ngdoc directive
 * @name bicycling-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <bicycling-layer></bicycling-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('bicyclingLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getLayer = function(options, events) {
    var layer = new google.maps.BicyclingLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }
    return layer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);

      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('bicyclingLayers', layer);
      parser.observeAttrSetObj(orgAttrs, attrs, layer);  //observers
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name cloud-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <cloud-layer></cloud-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('cloudLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getLayer = function(options, events) {
    var layer = new google.maps.weather.CloudLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }
    return layer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);
      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('cloudLayers', layer);
      parser.observeAttrSetObj(orgAttrs, attrs, layer);  //observers
    }
   }; // return
}]);

/*jshint -W030*/
/**
 * @ngdoc directive
 * @name custom-control
 * @requires Attr2Options 
 * @requires $compile
 * @description 
 *   Build custom control and set to the map with position
 *   
 *   Requires:  map directive
 *
 *   Restrict To:  Element
 *
 * @param {String} position position of this control
 *        i.e. TOP_RIGHT
 * @param {Number} index index of the control
 * @example
 *
 * Example: 
 *  <map center="41.850033,-87.6500523" zoom="3">
 *    <custom-control id="home" position="TOP_LEFT" index="1">
 *      <div style="background-color: white;">
 *        <b>Home</b>
 *      </div>
 *    </custom-control>
 *  </map>
 *
 */
/*jshint -W089*/
ngMap.directive('customControl', ['Attr2Options', '$compile', function(Attr2Options, $compile)  {
  var parser = Attr2Options;

  return {
    restrict: 'E',
    require: '^map',
    link: function(scope, element, attrs, mapController) {
      element.css('display','none');
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered, scope);
      var events = parser.getEvents(scope, filtered);
      void 0;

      /**
       * build a custom control element
       */
      var compiled = $compile(element.html().trim())(scope);
      var customControlEl = compiled[0];

      /**
       * set events
       */
      for (var eventName in events) {
        google.maps.event.addDomListener(customControlEl, eventName, events[eventName]);
      }

      mapController.addObject('customControls', customControlEl);
      scope.$on('mapInitialized', function(evt, map) {
        var position = options.position;
        map.controls[google.maps.ControlPosition[position]].push(customControlEl);
      });

    } //link
  }; // return
}]);// function

/**
 * @ngdoc directive
 * @name dynamic-maps-engine-layer
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *   <map zoom="14" center="[59.322506, 18.010025]">
 *     <dynamic-maps-engine-layer layer-id="06673056454046135537-08896501997766553811"></dynamic-maps-engine-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('dynamicMapsEngineLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;

  var getDynamicMapsEngineLayer = function(options, events) {
    var layer = new google.maps.visualization.DynamicMapsEngineLayer(options);

    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }

    return layer;
  };

  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered, events);
      void 0;

      var layer = getDynamicMapsEngineLayer(options, events);
      mapController.addObject('mapsEngineLayers', layer);
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name fusion-tables-layer
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *   <map zoom="11" center="41.850033, -87.6500523">
 *     <fusion-tables-layer query="{
 *       select: 'Geocodable address',
 *       from: '1mZ53Z70NsChnBMm-qEYmSDOvLXgrreLTkQUvvg'}">
 *     </fusion-tables-layer>
 *   </map>
 */
/*jshint -W089*/
ngMap.directive('fusionTablesLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;

  var getLayer = function(options, events) {
    var layer = new google.maps.FusionTablesLayer(options);

    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }

    return layer;
  };

  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered, events);
      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('fusionTablesLayers', layer);
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name heatmap-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="11" center="[41.875696,-87.624207]">
 *     <heatmap-layer data="taxiData"></heatmap-layer>
 *   </map>
 */
/*jshint -W089*/
ngMap.directive('heatmapLayer', ['Attr2Options', '$window', function(Attr2Options, $window) {
  var parser = Attr2Options;
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var filtered = parser.filter(attrs);

      /**
       * set options 
       */
      var options = parser.getOptions(filtered);
      options.data = $window[attrs.data] || scope[attrs.data];
      if (options.data instanceof Array) {
        options.data = new google.maps.MVCArray(options.data);
      } else {
        throw "invalid heatmap data";
      }
      var layer = new google.maps.visualization.HeatmapLayer(options);

      /**
       * set events 
       */
      var events = parser.getEvents(scope, filtered);
      void 0;

      mapController.addObject('heatmapLayers', layer);
    }
   }; // return
}]);

/*jshint -W030*/
/**
 * @ngdoc directive
 * @name info-window 
 * @requires Attr2Options 
 * @requires $compile
 * @description 
 *   Defines infoWindow and provides compile method
 *   
 *   Requires:  map directive
 *
 *   Restrict To:  Element
 *
 * @param {Boolean} visible Indicates to show it when map is initialized
 * @param {Boolean} visible-on-marker Indicates to show it on a marker when map is initialized
 * @param {String} &lt;InfoWindowOption> Any InfoWindow options,
 *        https://developers.google.com/maps/documentation/javascript/reference?csw=1#InfoWindowOptions  
 * @param {String} &lt;InfoWindowEvent> Any InfoWindow events, https://developers.google.com/maps/documentation/javascript/reference
 * @example
 * Usage: 
 *   <map MAP_ATTRIBUTES>
 *    <info-window id="foo" ANY_OPTIONS ANY_EVENTS"></info-window>
 *   </map>
 *
 * Example: 
 *  <map center="41.850033,-87.6500523" zoom="3">
 *    <info-window id="1" position="41.850033,-87.6500523" >
 *      <div ng-non-bindable>
 *        Chicago, IL<br/>
 *        LatLng: {{chicago.lat()}}, {{chicago.lng()}}, <br/>
 *        World Coordinate: {{worldCoordinate.x}}, {{worldCoordinate.y}}, <br/>
 *        Pixel Coordinate: {{pixelCoordinate.x}}, {{pixelCoordinate.y}}, <br/>
 *        Tile Coordinate: {{tileCoordinate.x}}, {{tileCoordinate.y}} at Zoom Level {{map.getZoom()}}
 *      </div>
 *    </info-window>
 *  </map>
 */
ngMap.directive('infoWindow', ['Attr2Options', '$compile', '$timeout', function(Attr2Options, $compile, $timeout)  {
  var parser = Attr2Options;

  var getInfoWindow = function(options, events, element) {
    var infoWindow;

    /**
     * set options
     */
    if (options.position && 
      !(options.position instanceof google.maps.LatLng)) {
      var address = options.position;
      delete options.position;
      infoWindow = new google.maps.InfoWindow(options);
      var callback = function() {
        infoWindow.open(infoWindow.map);
      }
      parser.setDelayedGeoLocation(infoWindow, 'setPosition', address, {callback: callback});
    } else {
      infoWindow = new google.maps.InfoWindow(options);
    }

    /**
     * set events
     */
    if (Object.keys(events).length > 0) {
      void 0;
    }
    for (var eventName in events) {
      if (eventName) {
        google.maps.event.addListener(infoWindow, eventName, events[eventName]);
      }
    }

    /**
     * set template ane template-relate functions
     * it must have a container element with ng-non-bindable
     */
    var template = element.html().trim();
    if (angular.element(template).length != 1) {
      throw "info-window working as a template must have a container";
    }
    infoWindow.__template = template.replace(/\s?ng-non-bindable[='"]+/,"");

    infoWindow.__compile = function(scope) {
      var el = $compile(infoWindow.__template)(scope);
      scope.$apply();
      infoWindow.setContent(el[0]);
    };

    infoWindow.__eval = function(event) {
      var template = infoWindow.__template;
      var _this = this;
      template = template.replace(/{{(event|this)[^;\}]+}}/g, function(match) {
        var expression = match.replace(/[{}]/g, "").replace("this.", "_this.");
        return eval(expression);
      });
      return template;
    };

    return infoWindow;
  };

  return {
    restrict: 'E',
    require: '^map',
    link: function(scope, element, attrs, mapController) {
      element.css('display','none');
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered, scope);
      var events = parser.getEvents(scope, filtered);
      void 0;

      var infoWindow = getInfoWindow(options, events, element);

      mapController.addObject('infoWindows', infoWindow);
      parser.observeAttrSetObj(orgAttrs, attrs, infoWindow); /* observers */

      // show InfoWindow when initialized
      if (infoWindow.visible) {
        //if (!infoWindow.position) { throw "Invalid position"; }
        scope.$on('mapInitialized', function(evt, map) {
          $timeout(function() {
            infoWindow.__template = infoWindow.__eval.apply(this, [evt]);
            infoWindow.__compile(scope);
            infoWindow.map = map;
            infoWindow.position && infoWindow.open(map);
          });
        });
      }

      // show InfoWindow on a marker  when initialized
      if (infoWindow.visibleOnMarker) {
        scope.$on('mapInitialized', function(evt, map) {
          $timeout(function() {
            var markerId = infoWindow.visibleOnMarker;
            var marker = map.markers[markerId];
            if (!marker) throw "Invalid marker id";
            infoWindow.__template = infoWindow.__eval.apply(this, [evt]);
            infoWindow.__compile(scope);
            infoWindow.open(map, marker);
          });
        });
      }

      /**
       * provide showInfoWindow method to scope
       */
      scope.showInfoWindow  = scope.showInfoWindow ||
        function(event, id, anchor) {
          var infoWindow = mapController.map.infoWindows[id];
          infoWindow.__template = infoWindow.__eval.apply(this, [event]);
          infoWindow.__compile(scope);
          if (anchor) {
            infoWindow.open(mapController.map, anchor);
          } else if (this.getPosition) {
            infoWindow.open(mapController.map, this);
          } else {
            infoWindow.open(mapController.map);
          }
        };

      /**
       * provide hideInfoWindow method to scope
       */
      scope.hideInfoWindow  = scope.hideInfoWindow ||
        function(event, id, anchor) {
          var infoWindow = mapController.map.infoWindows[id];
          infoWindow.__template = infoWindow.__eval.apply(this, [event]);
          infoWindow.__compile(scope);
          infoWindow.close();
        };

    } //link
  }; // return
}]);// function

/**
 * @ngdoc directive
 * @name kml-layer
 * @requires Attr2Options 
 * @description 
 *   renders Kml layer on a map
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @param {Url} url url of the kml layer
 * @param {KmlLayerOptions} KmlLayerOptions
 *   (https://developers.google.com/maps/documentation/javascript/reference#KmlLayerOptions)  
 * @param {String} &lt;KmlLayerEvent> Any KmlLayer events, https://developers.google.com/maps/documentation/javascript/reference
 * @example
 * Usage: 
 *   <map MAP_ATTRIBUTES>
 *    <kml-layer ANY_KML_LAYER ANY_KML_LAYER_EVENTS"></kml-layer>
 *   </map>
 *
 * Example: 
 *
 *   <map zoom="11" center="[41.875696,-87.624207]">
 *     <kml-layer url="http://gmaps-samples.googlecode.com/svn/trunk/ggeoxml/cta.kml" ></kml-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('kmlLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getKmlLayer = function(options, events) {
    var kmlLayer = new google.maps.KmlLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(kmlLayer, eventName, events[eventName]);
    }
    return kmlLayer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);
      void 0;

      var kmlLayer = getKmlLayer(options, events);
      mapController.addObject('kmlLayers', kmlLayer);
      parser.observeAttrSetObj(orgAttrs, attrs, kmlLayer);  //observers
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name map-data
 * @description 
 *   set map data
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @param {String} method-name, run map.data[method-name] with attribute value
 * @example
 * Example: 
 *
 *   <map zoom="11" center="[41.875696,-87.624207]">
 *     <map-data load-geo-json="https://storage.googleapis.com/maps-devrel/google.json"></map-data>
 *    </map>
 */
ngMap.directive('mapData', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered, events);

      void 0;
      scope.$on('mapInitialized', function(event, map) {
        /**
         * options
         */
        for (var key in options) {
          if (key) {
            var val = options[key];
            if (typeof scope[val] === "function") {
              map.data[key](scope[val]);
            } else {
              map.data[key](val);
            }
          } // if (key)
        }

        /**
         * events
         */
        for (var eventName in events) {
          if (events[eventName]) {
            map.data.addListener(eventName, events[eventName]);
          }
        }
      });
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name map-type
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <map-type name="coordinate" object="coordinateMapType"></map-type>
 *   </map>
 */
/*jshint -W089*/
ngMap.directive('mapType', ['Attr2Options', '$window', function(Attr2Options, $window) {
  var parser = Attr2Options;
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var mapTypeName = attrs.name, mapTypeObject;
      if (!mapTypeName) {
        throw "invalid map-type name";
      }
      if (attrs.object) {
        var __scope = scope[attrs.object] ? scope : $window;
        mapTypeObject = __scope[attrs.object];
        if (typeof mapTypeObject == "function") {
          mapTypeObject = new mapTypeObject();
        }
      }
      if (!mapTypeObject) {
        throw "invalid map-type object";
      }

      scope.$on('mapInitialized', function(evt, map) {
        map.mapTypes.set(mapTypeName, mapTypeObject);
      });
      mapController.addObject('mapTypes', mapTypeObject);
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name map
 * @requires Attr2Options
 * @description
 *   Implementation of {@link MapController}
 *   Initialize a Google map within a `<div>` tag with given options and register events
 *   It accepts children directives; marker, shape, or marker-clusterer
 *
 *   It initialize map, children tags, then emits message as soon as the action is done
 *   The message emitted from this directive is;
 *     . mapInitialized
 *
 *   Restrict To:
 *     Element
 *
 * @param {Array} geo-fallback-center 
 *    The center of map incase geolocation failed. i.e. [0,0]
 * @param {String} init-event The name of event to initialize this map. 
 *        If this option is given, the map won't be initialized until the event is received.
 *        To invoke the event, use $scope.$emit or $scope.$broacast. 
 *        i.e. <map init-event="init-map" ng-click="$emit('init-map')" center=... ></map>
 * @param {String} &lt;MapOption> Any Google map options, 
 *        https://developers.google.com/maps/documentation/javascript/reference?csw=1#MapOptions
 * @param {String} &lt;MapEvent> Any Google map events, 
 *        https://rawgit.com/allenhwkim/angularjs-google-maps/master/build/map_events.html
 * @example
 * Usage:
 *   <map MAP_OPTIONS_OR_MAP_EVENTS ..>
 *     ... Any children directives
 *   </map>
 * 
 * Example:
 *   <map center="[40.74, -74.18]" on-click="doThat()">
 *   </map>
 *
 *   <map geo-fallback-center="[40.74, -74.18]">
 *   </map>
 */
/*jshint -W030*/
ngMap.directive('map', ['Attr2Options', '$timeout', function(Attr2Options, $timeout) {
  var parser = Attr2Options;
  function getStyle(el,styleProp)
  {
    if (el.currentStyle) {
      var y = el.currentStyle[styleProp];
    } else if (window.getComputedStyle) {
      var y = document.defaultView.getComputedStyle(el,null).getPropertyValue(styleProp);
    }
    return y;
  }

  return {
    restrict: 'AE',
    controller: ngMap.MapController,
    /**
     * Initialize map and events
     * @memberof map
     * @param {$scope} scope
     * @param {angular.element} element
     * @param {Hash} attrs
     * @ctrl {MapController} ctrl
     */
    link: function (scope, element, attrs, ctrl) {
      var orgAttrs = parser.orgAttributes(element);

      scope.google = google;  //used by $scope.eval in Attr2Options to avoid eval()

      /**
       * create a new `div` inside map tag, so that it does not touch map element
       * http://stackoverflow.com/questions/20955356
       */
      var el = document.createElement("div");
      el.style.width = "100%";
      el.style.height = "100%";
      element.prepend(el);

      /**
       * if style is not given to the map element, set display and height
       */
      if (getStyle(element[0], 'display') != "block") {
        element.css('display','block');
      }
      if (getStyle(element[0], 'height').match(/^(0|auto)/)) {
        element.css('height','300px');
      }

      /**
       * initialize function
       */
      var initializeMap = function(mapOptions, mapEvents) {
        var map = new google.maps.Map(el, {});
        map.markers = {};
        map.shapes = {};
       
        /**
         * resize the map to prevent showing partially, in case intialized too early
         */
        $timeout(function() {
          google.maps.event.trigger(map, "resize");
        });

        /**
         * set options
         */
        mapOptions.zoom = mapOptions.zoom || 15;
        var center = mapOptions.center;
        if (!center) {
          mapOptions.center = new google.maps.LatLng(0,0);
        } else if (!(center instanceof google.maps.LatLng)) {
          delete mapOptions.center;
          Attr2Options.setDelayedGeoLocation(map, 'setCenter', 
              center, {fallbackLocation: options.geoFallbackCenter});
        }
        map.setOptions(mapOptions);

        /**
         * set events
         */
        for (var eventName in mapEvents) {
          if (eventName) {
            google.maps.event.addListener(map, eventName, mapEvents[eventName]);
          }
        }

        /**
         * set observers
         */
        parser.observeAttrSetObj(orgAttrs, attrs, map);

        /**
         * set controller and set objects
         * so that map can be used by other directives; marker or shape 
         * ctrl._objects are gathered when marker and shape are initialized before map is set
         */
        ctrl.map = map;   /* so that map can be used by other directives; marker or shape */
        ctrl.addObjects(ctrl._objects);

        // /* providing method to add a marker used by user scope */
        // map.addMarker = ctrl.addMarker;

        /**
         * set map for scope and controller and broadcast map event
         * scope.map will be overwritten if user have multiple maps in a scope,
         * thus the last map will be set as scope.map.
         * however an `mapInitialized` event will be emitted every time.
         */
        scope.map = map;
        scope.map.scope = scope;
        //google.maps.event.addListenerOnce(map, "idle", function() {
        scope.$emit('mapInitialized', map);  
        //});

        // the following lines will be deprecated on behalf of mapInitialized
        // to collect maps, we should use scope.maps in your own controller, i.e. MyCtrl
        scope.maps = scope.maps || {}; 
        scope.maps[options.id||Object.keys(scope.maps).length] = map;
        scope.$emit('mapsInitialized', scope.maps);  
      }; // function initializeMap()

      /**
       * get map options and events
       */
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered, scope);
      var controlOptions = parser.getControlOptions(filtered);
      var mapOptions = angular.extend(options, controlOptions);
      var mapEvents = parser.getEvents(scope, filtered);
      void 0;

      if (attrs.initEvent) { // allows controlled initialization
        scope.$on(attrs.initEvent, function() {
          !ctrl.map && initializeMap(mapOptions, mapEvents); // init if not done
        });
      } else {
        initializeMap(mapOptions, mapEvents);
      } // if
    }
  }; 
}]);

/**
 * @ngdoc directive
 * @name MapController
 * @requires $scope
 * @property {Hash} controls collection of Controls initiated within `map` directive
 * @property {Hash} markersi collection of Markers initiated within `map` directive
 * @property {Hash} shapes collection of shapes initiated within `map` directive
 * @property {MarkerClusterer} markerClusterer MarkerClusterer initiated within `map` directive
 */
/*jshint -W089*/
ngMap.MapController = function() { 

  this.map = null;
  this._objects = [];

  /**
   * Add a marker to map and $scope.markers
   * @memberof MapController
   * @name addMarker
   * @param {Marker} marker google map marker
   */
  this.addMarker = function(marker) {
    /**
     * marker and shape are initialized before map is initialized
     * so, collect _objects then will init. those when map is initialized
     * However the case as in ng-repeat, we can directly add to map
     */
    if (this.map) {
      this.map.markers = this.map.markers || {};
      marker.setMap(this.map);
      if (marker.centered) {
        this.map.setCenter(marker.position);
      }
      var len = Object.keys(this.map.markers).length;
      this.map.markers[marker.id || len] = marker;
    } else {
      this._objects.push(marker);
    }
  };

  /**
   * Add a shape to map and $scope.shapes
   * @memberof MapController
   * @name addShape
   * @param {Shape} shape google map shape
   */
  this.addShape = function(shape) {
    if (this.map) {
      this.map.shapes = this.map.shapes || {};
      shape.setMap(this.map);
      var len = Object.keys(this.map.shapes).length;
      this.map.shapes[shape.id || len] = shape;
    } else {
      this._objects.push(shape);
    }
  };

  this.addObject = function(groupName, obj) {
    if (this.map) {
      this.map[groupName] = this.map[groupName] || {};
      var len = Object.keys(this.map[groupName]).length;
      this.map[groupName][obj.id || len] = obj;
      if (groupName != "infoWindows" && obj.setMap) { //infoWindow.setMap works like infoWindow.open
        obj.setMap(this.map);
      }
    } else {
      obj.groupName = groupName;
      this._objects.push(obj);
    }
  }

  /**
   * Add a shape to map and $scope.shapes
   * @memberof MapController
   * @name addShape
   * @param {Shape} shape google map shape
   */
  this.addObjects = function(objects) {
    for (var i=0; i<objects.length; i++) {
      var obj=objects[i];
      if (obj instanceof google.maps.Marker) {
        this.addMarker(obj);
      } else if (obj instanceof google.maps.Circle ||
        obj instanceof google.maps.Polygon ||
        obj instanceof google.maps.Polyline ||
        obj instanceof google.maps.Rectangle ||
        obj instanceof google.maps.GroundOverlay) {
        this.addShape(obj);
      } else {
        this.addObject(obj.groupName, obj);
      }
    }
  };

};

/**
 * @ngdoc directive
 * @name maps-engine-layer
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *   <map zoom="14" center="[59.322506, 18.010025]">
 *     <maps-engine-layer layer-id="06673056454046135537-08896501997766553811"></maps-engine-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('mapsEngineLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;

  var getMapsEngineLayer = function(options, events) {
    var layer = new google.maps.visualization.MapsEngineLayer(options);

    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }

    return layer;
  };

  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered, events);
      void 0;

      var layer = getMapsEngineLayer(options, events);
      mapController.addObject('mapsEngineLayers', layer);
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name marker
 * @requires Attr2Options 
 * @requires NavigatorGeolocation
 * @description 
 *   Draw a Google map marker on a map with given options and register events  
 *   
 *   Requires:  map directive
 *
 *   Restrict To:  Element 
 *
 * @param {String} position address, 'current', or [latitude, longitude]  
 *    example:  
 *      '1600 Pennsylvania Ave, 20500  Washingtion DC',   
 *      'current position',  
 *      '[40.74, -74.18]'  
 * @param {Boolean} centered if set, map will be centered with this marker
 * @param {String} &lt;MarkerOption> Any Marker options, https://developers.google.com/maps/documentation/javascript/reference?csw=1#MarkerOptions  
 * @param {String} &lt;MapEvent> Any Marker events, https://developers.google.com/maps/documentation/javascript/reference
 * @example
 * Usage: 
 *   <map MAP_ATTRIBUTES>
 *    <marker ANY_MARKER_OPTIONS ANY_MARKER_EVENTS"></MARKER>
 *   </map>
 *
 * Example: 
 *   <map center="[40.74, -74.18]">
 *    <marker position="[40.74, -74.18]" on-click="myfunc()"></div>
 *   </map>
 *
 *   <map center="the cn tower">
 *    <marker position="the cn tower" on-click="myfunc()"></div>
 *   </map>
 */
ngMap.directive('marker', ['Attr2Options', function(Attr2Options)  {
  var parser = Attr2Options;

  var getMarker = function(options, events) {
    var marker;

    /**
     * set options
     */
    if (options.icon instanceof Object) {
      if ((""+options.icon.path).match(/^[A-Z_]+$/)) {
        options.icon.path =  google.maps.SymbolPath[options.icon.path];
      }
      for (var key in options.icon) {
        var arr = options.icon[key];
        if (key == "anchor" || key == "origin") {
          options.icon[key] = new google.maps.Point(arr[0], arr[1]);
        } else if (key == "size" || key == "scaledSize") {
          options.icon[key] = new google.maps.Size(arr[0], arr[1]);
        } 
      }
    }
    if (!(options.position instanceof google.maps.LatLng)) {
      var orgPosition = options.position;
      options.position = new google.maps.LatLng(0,0);
      marker = new google.maps.Marker(options);
      parser.setDelayedGeoLocation(marker, 'setPosition', orgPosition);
    } else {
      marker = new google.maps.Marker(options);
    }

    /**
     * set events
     */
    if (Object.keys(events).length > 0) {
      void 0;
    }
    for (var eventName in events) {
      if (eventName) {
        google.maps.event.addListener(marker, eventName, events[eventName]);
      }
    }

    return marker;
  };

  return {
    restrict: 'E',
    require: '^map',
    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var markerOptions = parser.getOptions(filtered, scope);
      var markerEvents = parser.getEvents(scope, filtered);
      void 0;

      /**
       * set event to clean up removed marker
       * useful with ng-repeat
       */
      element.bind('$destroy', function() {
        var markers = marker.map.markers;
        for (var name in markers) {
          if (markers[name] == marker) {
            delete markers[name];
          }
        }
        marker.setMap(null);          
      });

      var marker = getMarker(markerOptions, markerEvents);
      mapController.addMarker(marker);

      /**
       * set observers
       */
      parser.observeAttrSetObj(orgAttrs, attrs, marker); /* observers */

    } //link
  }; // return
}]);// 

/**
 * @ngdoc directive
 * @name overlay-map-type
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <overlay-map-type index="0" object="coordinateMapType"></map-type>
 *   </map>
 */
/*jshint -W089*/
ngMap.directive('overlayMapType', ['Attr2Options', '$window', function(Attr2Options, $window) {
  var parser = Attr2Options;
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var overlayMapTypeObject;
      var initMethod = attrs.initMethod || "insertAt";
      if (attrs.object) {
        var __scope = scope[attrs.object] ? scope : $window;
        overlayMapTypeObject = __scope[attrs.object];
        if (typeof overlayMapTypeObject == "function") {
          overlayMapTypeObject = new overlayMapTypeObject();
        }
      }
      if (!overlayMapTypeObject) {
        throw "invalid map-type object";
      }

      scope.$on('mapInitialized', function(evt, map) {
        if (initMethod == "insertAt") {
          var index = parseInt(attrs.index, 10);
          map.overlayMapTypes.insertAt(index, overlayMapTypeObject);
        } else if (initMethod == "push") {
          map.overlayMapTypes.push(overlayMapTypeObject);
        }
      });
      mapController.addObject('overlayMapTypes', overlayMapTypeObject);
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name shape
 * @requires Attr2Options 
 * @description 
 *   Initialize a Google map shape in map with given options and register events  
 *   The shapes are:
 *     . circle
 *     . polygon
 *     . polyline
 *     . rectangle
 *     . groundOverlay(or image)
 *   
 *   Requires:  map directive
 *
 *   Restrict To:  Element
 *
 * @param {Boolean} centered if set, map will be centered with this marker
 * @param {String} &lt;OPTIONS>
 *   For circle, [any circle options](https://developers.google.com/maps/documentation/javascript/reference#CircleOptions)  
 *   For polygon, [any polygon options](https://developers.google.com/maps/documentation/javascript/reference#PolygonOptions)  
 *   For polyline, [any polyline options](https://developers.google.com/maps/documentation/javascript/reference#PolylineOptions)   
 *   For rectangle, [any rectangle options](https://developers.google.com/maps/documentation/javascript/reference#RectangleOptions)   
 *   For image, [any groundOverlay options](https://developers.google.com/maps/documentation/javascript/reference#GroundOverlayOptions)   
 * @param {String} &lt;MapEvent> Any Shape events, https://developers.google.com/maps/documentation/javascript/reference
 * @example
 * Usage: 
 *   <map MAP_ATTRIBUTES>
 *    <shape name=SHAPE_NAME ANY_SHAPE_OPTIONS ANY_SHAPE_EVENTS"></MARKER>
 *   </map>
 *
 * Example: 
 *
 *   <map zoom="11" center="[40.74, -74.18]">
 *     <shape id="polyline" name="polyline" geodesic="true" stroke-color="#FF0000" stroke-opacity="1.0" stroke-weight="2"
 *      path="[[40.74,-74.18],[40.64,-74.10],[40.54,-74.05],[40.44,-74]]" ></shape>
 *    </map>
 *
 *   <map zoom="11" center="[40.74, -74.18]">
 *     <shape id="polygon" name="polygon" stroke-color="#FF0000" stroke-opacity="1.0" stroke-weight="2"
 *      paths="[[40.74,-74.18],[40.64,-74.18],[40.84,-74.08],[40.74,-74.18]]" ></shape>
 *   </map>
 *   
 *   <map zoom="11" center="[40.74, -74.18]">
 *     <shape id="rectangle" name="rectangle" stroke-color='#FF0000' stroke-opacity="0.8" stroke-weight="2"
 *      bounds="[[40.74,-74.18], [40.78,-74.14]]" editable="true" ></shape>
 *   </map>
 *
 *   <map zoom="11" center="[40.74, -74.18]">
 *     <shape id="circle" name="circle" stroke-color='#FF0000' stroke-opacity="0.8"stroke-weight="2" 
 *      center="[40.70,-74.14]" radius="4000" editable="true" ></shape>
 *   </map>
 *
 *   <map zoom="11" center="[40.74, -74.18]">
 *     <shape id="image" name="image" url="https://www.lib.utexas.edu/maps/historical/newark_nj_1922.jpg"
 *      bounds="[[40.71,-74.22],[40.77,-74.12]]" opacity="0.7" clickable="true" ></shape>
 *   </map>
 *
 *  For full-working example, please visit 
 *    [shape example](https://rawgit.com/allenhwkim/angularjs-google-maps/master/build/shape.html)
 */
ngMap.directive('shape', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getBounds = function(points) {
    return new google.maps.LatLngBounds(points[0], points[1]);
  };
  
  var getShape = function(options, events) {
    var shape;

    var shapeName = options.name;
    delete options.name;  //remove name bcoz it's not for options
    void 0;

    /**
     * set options
     */
    if (options.icons) {
      for (var i=0; i<options.icons.length; i++) {
        var el = options.icons[i];
        if (el.icon.path.match(/^[A-Z_]+$/)) {
          el.icon.path =  google.maps.SymbolPath[el.icon.path];
        }
      }
    }
    switch(shapeName) {
      case "circle":
        if (options.center instanceof google.maps.LatLng) {
          shape = new google.maps.Circle(options);
        } else {
          var orgCenter = options.center;
          options.center = new google.maps.LatLng(0,0);
          shape = new google.maps.Circle(options);
          parser.setDelayedGeoLocation(shape, 'setCenter', orgCenter);
        }
        break;
      case "polygon":
        shape = new google.maps.Polygon(options);
        break;
      case "polyline": 
        shape = new google.maps.Polyline(options);
        break;
      case "rectangle": 
        if (options.bounds) {
          options.bounds = getBounds(options.bounds);
        }
        shape = new google.maps.Rectangle(options);
        break;
      case "groundOverlay":
      case "image":
        var url = options.url;
        var bounds = getBounds(options.bounds);
        var opts = {opacity: options.opacity, clickable: options.clickable, id:options.id};
        shape = new google.maps.GroundOverlay(url, bounds, opts);
        break;
    }

    /**
     * set events
     */
    for (var eventName in events) {
      if (events[eventName]) {
        google.maps.event.addListener(shape, eventName, events[eventName]);
      }
    }
    return shape;
  };
  
  return {
    restrict: 'E',
    require: '^map',
    /**
     * link function
     * @private
     */
    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var shapeOptions = parser.getOptions(filtered);
      var shapeEvents = parser.getEvents(scope, filtered);

      var shape = getShape(shapeOptions, shapeEvents);
      mapController.addShape(shape);

      /**
       * set observers
       */
      parser.observeAttrSetObj(orgAttrs, attrs, shape); 
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name traffic-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <traffic-layer></traffic-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('trafficLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getLayer = function(options, events) {
    var layer = new google.maps.TrafficLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }
    return layer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);
      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('trafficLayers', layer);
      parser.observeAttrSetObj(orgAttrs, attrs, layer);  //observers
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name transit-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <transit-layer></transit-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('transitLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getLayer = function(options, events) {
    var layer = new google.maps.TransitLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }
    return layer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);
      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('transitLayers', layer);
      parser.observeAttrSetObj(orgAttrs, attrs, layer);  //observers
    }
   }; // return
}]);

/**
 * @ngdoc directive
 * @name weather-layer
 * @requires Attr2Options 
 * @description 
 *   Requires:  map directive
 *   Restrict To:  Element
 *
 * @example
 * Example: 
 *
 *   <map zoom="13" center="34.04924594193164, -118.24104309082031">
 *     <weather-layer></weather-layer>
 *    </map>
 */
/*jshint -W089*/
ngMap.directive('weatherLayer', ['Attr2Options', function(Attr2Options) {
  var parser = Attr2Options;
  
  var getLayer = function(options, events) {
    var layer = new google.maps.weather.WeatherLayer(options);
    for (var eventName in events) {
      google.maps.event.addListener(layer, eventName, events[eventName]);
    }
    return layer;
  };
  
  return {
    restrict: 'E',
    require: '^map',

    link: function(scope, element, attrs, mapController) {
      var orgAttrs = parser.orgAttributes(element);
      var filtered = parser.filter(attrs);
      var options = parser.getOptions(filtered);
      var events = parser.getEvents(scope, filtered);

      void 0;

      var layer = getLayer(options, events);
      mapController.addObject('weatherLayers', layer);
      parser.observeAttrSetObj(orgAttrs, attrs, layer);  //observers
    }
   }; // return
}]);
