'use strict';

/* Services */


// Demonstrate how to register services
// In this case it is a simple value service.
angular.module('pprzmon.services', []).service( 'modalService', ['$modal',

    ]).value('version', '0.1');

