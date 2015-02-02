'use strict';

// Declare app level module which depends on filters, and services
angular.module('pprzmon', [
  'ngCookies',
  'pprzmon.filters',
  'pprzmon.services',
  'pprzmon.directives',
  'pprzmon.controllers',
  'uiGmapgoogle-maps',
  'ngStorage',
  'ui.bootstrap',
  'btford.socket-io'
]).config(['$logProvider', '$interpolateProvider', '$httpProvider', function($logProvider, $interpolateProvider, $httpProvider){
      $logProvider.debugEnabled(true);
      $interpolateProvider.startSymbol('{[{');
      $interpolateProvider.endSymbol('}]}');
      $httpProvider.defaults.xsrfCookieName = 'csrftoken';
      $httpProvider.defaults.xsrfHeaderName = 'X-CSRFToken';
}]).config(['uiGmapGoogleMapApiProvider', function (GoogleMapApi) {
    GoogleMapApi.configure({
      // key: 'your api key',
        v: '3.17',
        libraries: 'weather,geometry,visualization'
    });
}]).factory('socket', function ($rootScope) {
  var socket = io.connect('http://localhost:3000');
  return {
    on: function (eventName, callback) {
      socket.on(eventName, function () {  
        var args = arguments;
        $rootScope.$apply(function () {
          callback.apply(socket, args);
        });
      });
    },
    emit: function (eventName, data, callback) {
      socket.emit(eventName, data, function () {
        var args = arguments;
        $rootScope.$apply(function () {
          if (callback) {
            callback.apply(socket, args);
          }
        });
      })
    }
  };
}).run( ['$http', '$cookies', function run( $http, $cookies ){
    // For CSRF token compatibility with Django
    $http.defaults.headers.post['X-CSRFToken'] = $cookies['csrftoken'];
}] );

//config(['$routeProvider', function($routeProvider) {
//  $routeProvider.when('/', {templateUrl: 'static/partials/main.html', controller: 'MainController'});
//  $routeProvider.when('/view2', {templateUrl: 'static/partials/partial2.html', controller: 'MyCtrl2'});
//  $routeProvider.otherwise({redirectTo: '/'});
//}]);
