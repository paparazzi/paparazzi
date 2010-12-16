var timeout = 100;

for( var i = 0; i < 100; i++ )
{
    eval("var timeoutli" + i + " = false;");
}


function initMenu2(){
    if ( browser.isDOM1 
    && !( browser.isMac && browser.isIE ) 
    && !( browser.isOpera && browser.versionMajor < 7 )
    && !( browser.isIE && browser.versionMajor < 5 ) )
    {
        var menu2 = document.getElementById('menu2'); 
        var lis2 = menu2.getElementsByTagName('li'); 

        menu2.className='menu2';     
        for ( var i=0; i<lis2.length; i++ )
        {
           
            if ( lis2.item(i).getElementsByTagName('ul').length > 0 )
            {        
                if ( browser.isIE )
                {
                    addAnEvent(lis2.item(i),'keyup',show);
                }
                
                addAnEvent(lis2.item(i),'mouseover',show);
                addAnEvent(lis2.item(i),'mouseout',timeoutHide);
                addAnEvent(lis2.item(i),'blur',timeoutHide);
                addAnEvent(lis2.item(i),'focus',show);
                
                lis2.item(i).setAttribute( 'id', "li2"+i );
            }
        }
    }
}



function addAnEvent( target, eventName, functionName )
{
    if ( browser.isIE )
    {
        eval('target.on'+eventName+'=functionName');
    }
	 else
    {
        target.addEventListener( eventName , functionName , true );
    }
}
    
function timeoutHide()
{
    eval( "timeout" + this.id + " = window.setTimeout('hideUlUnder( \"" + this.id + "\" )', " + timeout + " );");
}

// hide the ul elements under the element identified by id
function hideUlUnder( id )
{   
    document.getElementById(id).getElementsByTagName('ul')[0].style['visibility'] = 'hidden';
}

// show the first ul element found under this element
function show()
{
    // show the sub menu
    this.getElementsByTagName('ul')[0].style['visibility'] = 'visible';
    var currentNode=this;
    while(currentNode)
    {
            if( currentNode.nodeName=='LI')
            {
               // currentNode.getElementsByTagName('a')[0].className = 'linkOver';
            }
            currentNode=currentNode.parentNode;
    }
    // clear the timeout
    eval ( "clearTimeout( timeout"+ this.id +");" );
    hideAllOthersUls( this );
}

// hide all ul on the same level of  this list item
function hideAllOthersUls( currentLi )
{
    var lis = currentLi.parentNode;
    for ( var i=0; i<lis.childNodes.length; i++ )
    {
        if ( lis.childNodes[i].nodeName=='LI' && lis.childNodes[i].id != currentLi.id )
        {
            hideUlUnderLi( lis.childNodes[i] );
        }
    }
}

// hide all the ul wich are in the li element
function hideUlUnderLi( li )
{
    var as = li.getElementsByTagName('a');
    for ( var i=0; i<as.length; i++ )
    {
        as.item(i).className="";
    }
    var uls = li.getElementsByTagName('ul');
    for ( var i=0; i<uls.length; i++ )
    {
        uls.item(i).style['visibility'] = 'hidden';
    }
} 
