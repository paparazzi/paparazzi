// Browser Detect Lite  v2.1.4
// http://www.dithered.com/javascript/browser_detect/index.html
// modified by Chris Nott (chris@NOSPAMdithered.com - remove NOSPAM)


function BrowserDetectLite() {
   var ua = navigator.userAgent.toLowerCase(); 

   // browser name
   this.isGecko     = (ua.indexOf('gecko') != -1 && ua.indexOf('safari') == -1);
   this.isMozilla   = (this.isGecko && ua.indexOf('gecko/') + 14 == ua.length);
   this.isNS        = ( (this.isGecko) ? (ua.indexOf('netscape') != -1) : ( (ua.indexOf('mozilla') != -1) && (ua.indexOf('spoofer') == -1) && (ua.indexOf('compatible') == -1) && (ua.indexOf('opera') == -1) && (ua.indexOf('webtv') == -1) && (ua.indexOf('hotjava') == -1) ) );
   this.isIE        = ( (ua.indexOf('msie') != -1) && (ua.indexOf('opera') == -1) && (ua.indexOf('webtv') == -1) ); 
   this.isSafari    = (ua.indexOf('safari') != - 1);
   this.isOpera     = (ua.indexOf('opera') != -1); 
   this.isKonqueror = (ua.indexOf('konqueror') != -1 && !this.isSafari); 
   this.isIcab      = (ua.indexOf('icab') != -1); 
   this.isAol       = (ua.indexOf('aol') != -1); 
   
   // spoofing and compatible browsers
   this.isIECompatible = ( (ua.indexOf('msie') != -1) && !this.isIE);
   this.isNSCompatible = ( (ua.indexOf('mozilla') != -1) && !this.isNS && !this.isMozilla);
   
   // browser version
   this.versionMinor = parseFloat(navigator.appVersion); 
   
   // correct version number
   if (this.isNS && this.isGecko) {
      this.versionMinor = parseFloat( ua.substring( ua.lastIndexOf('/') + 1 ) );
   }
   else if (this.isIE && this.versionMinor >= 4) {
      this.versionMinor = parseFloat( ua.substring( ua.indexOf('msie ') + 5 ) );
   }
   else if (this.isMozilla) {
      this.versionMinor = parseFloat( ua.substring( ua.indexOf('rv:') + 3 ) );
   }
   else if (this.isSafari) {
      this.versionMinor = parseFloat( ua.substring( ua.lastIndexOf('/') + 1 ) );
   }
   else if (this.isOpera) {
      if (ua.indexOf('opera/') != -1) {
         this.versionMinor = parseFloat( ua.substring( ua.indexOf('opera/') + 6 ) );
      }
      else {
         this.versionMinor = parseFloat( ua.substring( ua.indexOf('opera ') + 6 ) );
      }
   }
   else if (this.isKonqueror) {
      this.versionMinor = parseFloat( ua.substring( ua.indexOf('konqueror/') + 10 ) );
   }
   else if (this.isIcab) {
      if (ua.indexOf('icab/') != -1) {
         this.versionMinor = parseFloat( ua.substring( ua.indexOf('icab/') + 6 ) );
      }
      else {
         this.versionMinor = parseFloat( ua.substring( ua.indexOf('icab ') + 6 ) );
      }
   }
   
   this.versionMajor = parseInt(this.versionMinor); 
   this.geckoVersion = ( (this.isGecko) ? ua.substring( (ua.lastIndexOf('gecko/') + 6), (ua.lastIndexOf('gecko/') + 14) ) : -1 );
   
   // dom support
   this.isDOM1 = (document.getElementById);
   this.isDOM2Event = (document.addEventListener && document.removeEventListener);
   
   // css compatibility mode
   this.mode = document.compatMode ? document.compatMode : 'BackCompat';

   // platform
   this.isWin   = (ua.indexOf('win') != -1);
   this.isWin32 = (this.isWin && ( ua.indexOf('95') != -1 || ua.indexOf('98') != -1 || ua.indexOf('nt') != -1 || ua.indexOf('win32') != -1 || ua.indexOf('32bit') != -1 || ua.indexOf('xp') != -1) );
   this.isMac   = (ua.indexOf('mac') != -1);
   this.isUnix  = (ua.indexOf('unix') != -1 || ua.indexOf('sunos') != -1 || ua.indexOf('bsd') != -1 || ua.indexOf('x11') != -1)
   this.isLinux = (ua.indexOf('linux') != -1);
   
   // specific browser shortcuts
   this.isNS4x = (this.isNS && this.versionMajor == 4);
   this.isNS40x = (this.isNS4x && this.versionMinor < 4.5);
   this.isNS47x = (this.isNS4x && this.versionMinor >= 4.7);
   this.isNS4up = (this.isNS && this.versionMinor >= 4);
   this.isNS6x = (this.isNS && this.versionMajor == 6);
   this.isNS6up = (this.isNS && this.versionMajor >= 6);
   this.isNS7x = (this.isNS && this.versionMajor == 7);
   this.isNS7up = (this.isNS && this.versionMajor >= 7);
   
   this.isIE4x = (this.isIE && this.versionMajor == 4);
   this.isIE4up = (this.isIE && this.versionMajor >= 4);
   this.isIE5x = (this.isIE && this.versionMajor == 5);
   this.isIE55 = (this.isIE && this.versionMinor == 5.5);
   this.isIE5up = (this.isIE && this.versionMajor >= 5);
   this.isIE6x = (this.isIE && this.versionMajor == 6);
   this.isIE6up = (this.isIE && this.versionMajor >= 6);
   
   this.isIE4xMac = (this.isIE4x && this.isMac);
}
var browser = new BrowserDetectLite();
