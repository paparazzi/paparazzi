function DOMImplementation(sUrl, fCallback) {
        var dom;
        if(window.ActiveXObject) {
                dom = new ActiveXObject("Microsoft.XMLDOM");
                dom.onreadystatechange = function() {
                        if(dom.readyState == 4) {
                        	   //alert("win "+sUrl+" loaded");
                        	   fCallback(dom);
                        }
                };
        }
        else if(document.implementation && document.implementation.createDocument) {
                dom = document.implementation.createDocument("", "", null);
                dom.onload = function() { 
                      //alert("other "+sUrl+" loaded"); 
                       fCallback(dom); 
                }
        }       
        else {
                alert("Votre navigateur ne gère pas l'importation de fichiers XML");
                return;
        } 
        dom.load(sUrl);
}
