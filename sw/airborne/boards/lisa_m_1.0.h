


<!DOCTYPE html>
<html>
  <head>
    <meta charset='utf-8'>
    <meta http-equiv="X-UA-Compatible" content="chrome=1">
        <title>sw/airborne/boards/lisa_m_1.0.h at master from paparazzi/paparazzi - GitHub</title>
    <link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="GitHub" />
    <link rel="fluid-icon" href="https://github.com/fluidicon.png" title="GitHub" />

    
    

    <meta content="authenticity_token" name="csrf-param" />
<meta content="xYeJ1/oDiWokRuiJrFjdWpya2LbyMaZGVwDyZBSpTNo=" name="csrf-token" />

    <link href="https://a248.e.akamai.net/assets.github.com/stylesheets/bundles/github-7026fab394f807b8901014695fb0d36c9be30654.css" media="screen" rel="stylesheet" type="text/css" />
    

    <script src="https://a248.e.akamai.net/assets.github.com/javascripts/bundles/jquery-e46225f266eba00902b2e5b66fe6fe6a484fb242.js" type="text/javascript"></script>
    <script src="https://a248.e.akamai.net/assets.github.com/javascripts/bundles/github-5437ec7baf7b1e27c7e55de1efad59c3d7bfa433.js" type="text/javascript"></script>
    

      <link rel='permalink' href='/paparazzi/paparazzi/blob/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw/airborne/boards/lisa_m_1.0.h'>

    <meta name="description" content="paparazzi - Paparazzi is a free and open-source hardware and software project for unmanned (air) vehicles. This is the main software repository." />
  <link href="https://github.com/paparazzi/paparazzi/commits/master.atom" rel="alternate" title="Recent Commits to paparazzi:master" type="application/atom+xml" />

  </head>


  <body class="logged_out page-blob linux  env-production ">
    


    

      <div id="header" class="true clearfix">
        <div class="container clearfix">
          <a class="site-logo" href="https://github.com">
            <!--[if IE]>
            <img alt="GitHub" class="github-logo" src="https://a248.e.akamai.net/assets.github.com/images/modules/header/logov7.png?1323882728" />
            <img alt="GitHub" class="github-logo-hover" src="https://a248.e.akamai.net/assets.github.com/images/modules/header/logov7-hover.png?1324325369" />
            <![endif]-->
            <img alt="GitHub" class="github-logo-4x" height="30" src="https://a248.e.akamai.net/assets.github.com/images/modules/header/logov7@4x.png?1323882728" />
            <img alt="GitHub" class="github-logo-4x-hover" height="30" src="https://a248.e.akamai.net/assets.github.com/images/modules/header/logov7@4x-hover.png?1324325369" />
          </a>

                  <!--
      make sure to use fully qualified URLs here since this nav
      is used on error pages on other domains
    -->
    <ul class="top-nav logged_out">
        <li class="pricing"><a href="https://github.com/plans">Signup and Pricing</a></li>
        <li class="explore"><a href="https://github.com/explore">Explore GitHub</a></li>
      <li class="features"><a href="https://github.com/features">Features</a></li>
        <li class="blog"><a href="https://github.com/blog">Blog</a></li>
      <li class="login"><a href="https://github.com/login?return_to=%2Fpaparazzi%2Fpaparazzi%2Fblob%2Fmaster%2Fsw%2Fairborne%2Fboards%2Flisa_m_1.0.h">Login</a></li>
    </ul>



          
        </div>
      </div>

      

            <div class="site">
      <div class="container">
        <div class="pagehead repohead instapaper_ignore readability-menu">


        <div class="title-actions-bar">
          <h1>
            <a href="/paparazzi">paparazzi</a> /
            <strong><a href="/paparazzi/paparazzi" class="js-current-repository">paparazzi</a></strong>
          </h1>
          



              <ul class="pagehead-actions">


          <li><a href="/login?return_to=%2Fpaparazzi%2Fpaparazzi" class="minibutton btn-watch watch-button entice tooltipped leftwards" rel="nofollow" title="You must be logged in to use this feature"><span><span class="icon"></span>Watch</span></a></li>
          <li><a href="/login?return_to=%2Fpaparazzi%2Fpaparazzi" class="minibutton btn-fork fork-button entice tooltipped leftwards" rel="nofollow" title="You must be logged in to use this feature"><span><span class="icon"></span>Fork</span></a></li>


      <li class="repostats">
        <ul class="repo-stats">
          <li class="watchers ">
            <a href="/paparazzi/paparazzi/watchers" title="Watchers" class="tooltipped downwards">
              134
            </a>
          </li>
          <li class="forks">
            <a href="/paparazzi/paparazzi/network" title="Forks" class="tooltipped downwards">
              88
            </a>
          </li>
        </ul>
      </li>
    </ul>

        </div>

          

  <ul class="tabs">
    <li><a href="/paparazzi/paparazzi" class="selected" highlight="repo_sourcerepo_downloadsrepo_commitsrepo_tagsrepo_branches">Code</a></li>
    <li><a href="/paparazzi/paparazzi/network" highlight="repo_networkrepo_fork_queue">Network</a>
    <li><a href="/paparazzi/paparazzi/pulls" highlight="repo_pulls">Pull Requests <span class='counter'>4</span></a></li>

      <li><a href="/paparazzi/paparazzi/issues" highlight="repo_issues">Issues <span class='counter'>35</span></a></li>


    <li><a href="/paparazzi/paparazzi/graphs" highlight="repo_graphsrepo_contributors">Stats &amp; Graphs</a></li>

  </ul>

  
<div class="frame frame-center tree-finder" style="display:none"
      data-tree-list-url="/paparazzi/paparazzi/tree-list/0ed489b9d13c4476517d632660db4e31e1b6e92b"
      data-blob-url-prefix="/paparazzi/paparazzi/blob/0ed489b9d13c4476517d632660db4e31e1b6e92b"
    >

  <div class="breadcrumb">
    <b><a href="/paparazzi/paparazzi">paparazzi</a></b> /
    <input class="tree-finder-input js-navigation-enable" type="text" name="query" autocomplete="off" spellcheck="false">
  </div>

    <div class="octotip">
      <p>
        <a href="/paparazzi/paparazzi/dismiss-tree-finder-help" class="dismiss js-dismiss-tree-list-help" title="Hide this notice forever" rel="nofollow">Dismiss</a>
        <strong>Octotip:</strong> You've activated the <em>file finder</em>
        by pressing <span class="kbd">t</span> Start typing to filter the
        file list. Use <span class="kbd badmono">↑</span> and
        <span class="kbd badmono">↓</span> to navigate,
        <span class="kbd">enter</span> to view files.
      </p>
    </div>

  <table class="tree-browser" cellpadding="0" cellspacing="0">
    <tr class="js-header"><th>&nbsp;</th><th>name</th></tr>
    <tr class="js-no-results no-results" style="display: none">
      <th colspan="2">No matching files</th>
    </tr>
    <tbody class="js-results-list js-navigation-container" data-navigation-enable-mouse>
    </tbody>
  </table>
</div>

<div id="jump-to-line" style="display:none">
  <h2>Jump to Line</h2>
  <form>
    <input class="textfield" type="text">
    <div class="full-button">
      <button type="submit" class="classy">
        <span>Go</span>
      </button>
    </div>
  </form>
</div>


<div class="subnav-bar">

  <ul class="actions">
    
      <li class="switcher">

        <div class="context-menu-container js-menu-container">
          <span class="text">Current branch:</span>
          <a href="#"
             class="minibutton bigger switcher context-menu-button js-menu-target js-commitish-button btn-branch repo-tree"
             data-master-branch="master"
             data-ref="master">
            <span><span class="icon"></span>master</span>
          </a>

          <div class="context-pane commitish-context js-menu-content">
            <a href="javascript:;" class="close js-menu-close"></a>
            <div class="context-title">Switch Branches/Tags</div>
            <div class="context-body pane-selector commitish-selector js-filterable-commitishes">
              <div class="filterbar">
                <div class="placeholder-field js-placeholder-field">
                  <label class="placeholder" for="context-commitish-filter-field" data-placeholder-mode="sticky">Filter branches/tags</label>
                  <input type="text" id="context-commitish-filter-field" class="commitish-filter" />
                </div>

                <ul class="tabs">
                  <li><a href="#" data-filter="branches" class="selected">Branches</a></li>
                  <li><a href="#" data-filter="tags">Tags</a></li>
                </ul>
              </div>

                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/aspirin2/sw/airborne/boards/lisa_m_1.0.h" data-name="aspirin2" rel="nofollow">aspirin2</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/aspirin_improved/sw/airborne/boards/lisa_m_1.0.h" data-name="aspirin_improved" rel="nofollow">aspirin_improved</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/campaign2010/sw/airborne/boards/lisa_m_1.0.h" data-name="campaign2010" rel="nofollow">campaign2010</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/dev/sw/airborne/boards/lisa_m_1.0.h" data-name="dev" rel="nofollow">dev</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/HB/sw/airborne/boards/lisa_m_1.0.h" data-name="HB" rel="nofollow">HB</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/master/sw/airborne/boards/lisa_m_1.0.h" data-name="master" rel="nofollow">master</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/new-state-interface/sw/airborne/boards/lisa_m_1.0.h" data-name="new-state-interface" rel="nofollow">new-state-interface</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/positive_control_gains/sw/airborne/boards/lisa_m_1.0.h" data-name="positive_control_gains" rel="nofollow">positive_control_gains</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/rotorcraft_positive_control_gains/sw/airborne/boards/lisa_m_1.0.h" data-name="rotorcraft_positive_control_gains" rel="nofollow">rotorcraft_positive_control_gains</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/sdlogging/sw/airborne/boards/lisa_m_1.0.h" data-name="sdlogging" rel="nofollow">sdlogging</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/stm_i2c/sw/airborne/boards/lisa_m_1.0.h" data-name="stm_i2c" rel="nofollow">stm_i2c</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/systime/sw/airborne/boards/lisa_m_1.0.h" data-name="systime" rel="nofollow">systime</a>
                  </h4>
                </div>
                <div class="commitish-item branch-commitish selector-item">
                  <h4>
                      <a href="/paparazzi/paparazzi/blob/systime_i2c/sw/airborne/boards/lisa_m_1.0.h" data-name="systime_i2c" rel="nofollow">systime_i2c</a>
                  </h4>
                </div>


              <div class="no-results" style="display:none">Nothing to show</div>
            </div>
          </div><!-- /.commitish-context-context -->
        </div>

      </li>
  </ul>

  <ul class="subnav">
    <li><a href="/paparazzi/paparazzi" class="selected" highlight="repo_source">Files</a></li>
    <li><a href="/paparazzi/paparazzi/commits/master" highlight="repo_commits">Commits</a></li>
    <li><a href="/paparazzi/paparazzi/branches" class="" highlight="repo_branches" rel="nofollow">Branches <span class="counter">13</span></a></li>
    <li><a href="/paparazzi/paparazzi/tags" class="blank" highlight="repo_tags">Tags <span class="counter">0</span></a></li>
    <li><a href="/paparazzi/paparazzi/downloads" class="blank" highlight="repo_downloads">Downloads <span class="counter">0</span></a></li>
  </ul>

</div>

  
  
  


          

        </div><!-- /.repohead -->

        




  
  <p class="last-commit">Latest commit to the <strong>master</strong> branch</p>

<div class="commit commit-tease js-details-container">
  <p class="commit-title ">
      <a href="/paparazzi/paparazzi/commit/0ed489b9d13c4476517d632660db4e31e1b6e92b" class="message">added eclipse project files to .gitignore</a>
      
  </p>
  <div class="commit-meta">
    <a href="/paparazzi/paparazzi/commit/0ed489b9d13c4476517d632660db4e31e1b6e92b" class="sha-block">commit <span class="sha">0ed489b9d1</span></a>

    <div class="authorship">
      <img class="gravatar" height="20" src="https://secure.gravatar.com/avatar/17a8aaf6d782b30b3d6011c2e173215b?s=140&amp;d=https://a248.e.akamai.net/assets.github.com%2Fimages%2Fgravatars%2Fgravatar-140.png" width="20" />
      <span class="author-name"><a href="/flixr">flixr</a></span>
      authored <time class="js-relative-date" datetime="2012-01-24T14:12:07-08:00" title="2012-01-24 14:12:07">January 24, 2012</time>

    </div>
  </div>
</div>


<!-- block_view_fragment_key: views4/v8/blob:v15:1068018:paparazzi/paparazzi:fbc668aae88e324bc693f2eebe8295f6eb15fd44:833a1d72767b16b8ab8f00d201f0ff87 -->
  <div id="slider">

    <div class="breadcrumb" data-path="sw/airborne/boards/lisa_m_1.0.h/">
      <b><a href="/paparazzi/paparazzi/tree/0ed489b9d13c4476517d632660db4e31e1b6e92b" class="js-rewrite-sha">paparazzi</a></b> / <a href="/paparazzi/paparazzi/tree/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw" class="js-rewrite-sha">sw</a> / <a href="/paparazzi/paparazzi/tree/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw/airborne" class="js-rewrite-sha">airborne</a> / <a href="/paparazzi/paparazzi/tree/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw/airborne/boards" class="js-rewrite-sha">boards</a> / lisa_m_1.0.h       <span style="display:none" id="clippy_2329" class="clippy-text">sw/airborne/boards/lisa_m_1.0.h</span>
      
      <object classid="clsid:d27cdb6e-ae6d-11cf-96b8-444553540000"
              width="110"
              height="14"
              class="clippy"
              id="clippy" >
      <param name="movie" value="https://a248.e.akamai.net/assets.github.com/flash/clippy.swf?1310086001?v5"/>
      <param name="allowScriptAccess" value="always" />
      <param name="quality" value="high" />
      <param name="scale" value="noscale" />
      <param NAME="FlashVars" value="id=clippy_2329&amp;copied=copied!&amp;copyto=copy to clipboard">
      <param name="bgcolor" value="#FFFFFF">
      <param name="wmode" value="opaque">
      <embed src="https://a248.e.akamai.net/assets.github.com/flash/clippy.swf?1310086001?v5"
             width="110"
             height="14"
             name="clippy"
             quality="high"
             allowScriptAccess="always"
             type="application/x-shockwave-flash"
             pluginspage="http://www.macromedia.com/go/getflashplayer"
             FlashVars="id=clippy_2329&amp;copied=copied!&amp;copyto=copy to clipboard"
             bgcolor="#FFFFFF"
             wmode="opaque"
      />
      </object>
      

    </div>

    <div class="frames">
      <div class="frame frame-center" data-path="sw/airborne/boards/lisa_m_1.0.h/" data-permalink-url="/paparazzi/paparazzi/blob/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw/airborne/boards/lisa_m_1.0.h" data-title="sw/airborne/boards/lisa_m_1.0.h at master from paparazzi/paparazzi - GitHub" data-type="blob">
          <ul class="big-actions">
            <li><a class="file-edit-link minibutton js-rewrite-sha" href="/paparazzi/paparazzi/edit/0ed489b9d13c4476517d632660db4e31e1b6e92b/sw/airborne/boards/lisa_m_1.0.h" data-method="post" rel="nofollow"><span>Edit this file</span></a></li>
          </ul>

        <div id="files" class="bubble">
          <div class="file">
            <div class="meta">
              <div class="info">
                <span class="icon"><img alt="Txt" height="16" src="https://a248.e.akamai.net/assets.github.com/images/icons/txt.png?1310086001" width="16" /></span>
                <span class="mode" title="File Mode">100644</span>
                  <span>74 lines (59 sloc)</span>
                <span>1.868 kb</span>
              </div>
              <ul class="actions">
                <li><a href="/paparazzi/paparazzi/raw/master/sw/airborne/boards/lisa_m_1.0.h" id="raw-url">raw</a></li>
                  <li><a href="/paparazzi/paparazzi/blame/master/sw/airborne/boards/lisa_m_1.0.h">blame</a></li>
                <li><a href="/paparazzi/paparazzi/commits/master/sw/airborne/boards/lisa_m_1.0.h" rel="nofollow">history</a></li>
              </ul>
            </div>
              <div class="data type-c">
      <table cellpadding="0" cellspacing="0" class="lines">
        <tr>
          <td>
            <pre class="line_numbers"><span id="L1" rel="#L1">1</span>
<span id="L2" rel="#L2">2</span>
<span id="L3" rel="#L3">3</span>
<span id="L4" rel="#L4">4</span>
<span id="L5" rel="#L5">5</span>
<span id="L6" rel="#L6">6</span>
<span id="L7" rel="#L7">7</span>
<span id="L8" rel="#L8">8</span>
<span id="L9" rel="#L9">9</span>
<span id="L10" rel="#L10">10</span>
<span id="L11" rel="#L11">11</span>
<span id="L12" rel="#L12">12</span>
<span id="L13" rel="#L13">13</span>
<span id="L14" rel="#L14">14</span>
<span id="L15" rel="#L15">15</span>
<span id="L16" rel="#L16">16</span>
<span id="L17" rel="#L17">17</span>
<span id="L18" rel="#L18">18</span>
<span id="L19" rel="#L19">19</span>
<span id="L20" rel="#L20">20</span>
<span id="L21" rel="#L21">21</span>
<span id="L22" rel="#L22">22</span>
<span id="L23" rel="#L23">23</span>
<span id="L24" rel="#L24">24</span>
<span id="L25" rel="#L25">25</span>
<span id="L26" rel="#L26">26</span>
<span id="L27" rel="#L27">27</span>
<span id="L28" rel="#L28">28</span>
<span id="L29" rel="#L29">29</span>
<span id="L30" rel="#L30">30</span>
<span id="L31" rel="#L31">31</span>
<span id="L32" rel="#L32">32</span>
<span id="L33" rel="#L33">33</span>
<span id="L34" rel="#L34">34</span>
<span id="L35" rel="#L35">35</span>
<span id="L36" rel="#L36">36</span>
<span id="L37" rel="#L37">37</span>
<span id="L38" rel="#L38">38</span>
<span id="L39" rel="#L39">39</span>
<span id="L40" rel="#L40">40</span>
<span id="L41" rel="#L41">41</span>
<span id="L42" rel="#L42">42</span>
<span id="L43" rel="#L43">43</span>
<span id="L44" rel="#L44">44</span>
<span id="L45" rel="#L45">45</span>
<span id="L46" rel="#L46">46</span>
<span id="L47" rel="#L47">47</span>
<span id="L48" rel="#L48">48</span>
<span id="L49" rel="#L49">49</span>
<span id="L50" rel="#L50">50</span>
<span id="L51" rel="#L51">51</span>
<span id="L52" rel="#L52">52</span>
<span id="L53" rel="#L53">53</span>
<span id="L54" rel="#L54">54</span>
<span id="L55" rel="#L55">55</span>
<span id="L56" rel="#L56">56</span>
<span id="L57" rel="#L57">57</span>
<span id="L58" rel="#L58">58</span>
<span id="L59" rel="#L59">59</span>
<span id="L60" rel="#L60">60</span>
<span id="L61" rel="#L61">61</span>
<span id="L62" rel="#L62">62</span>
<span id="L63" rel="#L63">63</span>
<span id="L64" rel="#L64">64</span>
<span id="L65" rel="#L65">65</span>
<span id="L66" rel="#L66">66</span>
<span id="L67" rel="#L67">67</span>
<span id="L68" rel="#L68">68</span>
<span id="L69" rel="#L69">69</span>
<span id="L70" rel="#L70">70</span>
<span id="L71" rel="#L71">71</span>
<span id="L72" rel="#L72">72</span>
<span id="L73" rel="#L73">73</span>
</pre>
          </td>
          <td width="100%">
                <div class="highlight"><pre><div class='line' id='LC1'><span class="cp">#ifndef CONFIG_LISA_M_1_0_H</span></div><div class='line' id='LC2'><span class="cp">#define CONFIG_LISA_M_1_0_H</span></div><div class='line' id='LC3'><br/></div><div class='line' id='LC4'><span class="cp">#define BOARD_LISA_M</span></div><div class='line' id='LC5'><br/></div><div class='line' id='LC6'><span class="cp">#define AHB_CLK 72000000</span></div><div class='line' id='LC7'><br/></div><div class='line' id='LC8'><span class="cp">/* Onboard LEDs */</span></div><div class='line' id='LC9'><span class="cp">#define LED_1_BANK</span></div><div class='line' id='LC10'><span class="cp">#define LED_1_GPIO GPIOB</span></div><div class='line' id='LC11'><span class="cp">#define LED_1_GPIO_CLK RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO</span></div><div class='line' id='LC12'><span class="cp">#define LED_1_GPIO_PIN GPIO_Pin_4</span></div><div class='line' id='LC13'><span class="cp">#define LED_1_AFIO_REMAP GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE)</span></div><div class='line' id='LC14'><br/></div><div class='line' id='LC15'><span class="cp">#define LED_2_BANK</span></div><div class='line' id='LC16'><span class="cp">#define LED_2_GPIO GPIOC</span></div><div class='line' id='LC17'><span class="cp">#define LED_2_GPIO_CLK RCC_APB2Periph_GPIOC</span></div><div class='line' id='LC18'><span class="cp">#define LED_2_GPIO_PIN GPIO_Pin_5</span></div><div class='line' id='LC19'><span class="cp">#define LED_2_AFIO_REMAP ((void)0)</span></div><div class='line' id='LC20'><br/></div><div class='line' id='LC21'><span class="cp">#define LED_3_BANK</span></div><div class='line' id='LC22'><span class="cp">#define LED_3_GPIO GPIOC</span></div><div class='line' id='LC23'><span class="cp">#define LED_3_GPIO_CLK RCC_APB2Periph_GPIOC</span></div><div class='line' id='LC24'><span class="cp">#define LED_3_GPIO_PIN GPIO_Pin_2</span></div><div class='line' id='LC25'><span class="cp">#define LED_3_AFIO_REMAP ((void)0)</span></div><div class='line' id='LC26'><br/></div><div class='line' id='LC27'><br/></div><div class='line' id='LC28'><span class="cp">/* configuration for aspirin - and more generaly IMUs */</span></div><div class='line' id='LC29'><span class="cp">#define IMU_ACC_DRDY_RCC_GPIO         RCC_APB2Periph_GPIOB</span></div><div class='line' id='LC30'><span class="cp">#define IMU_ACC_DRDY_GPIO             GPIOB</span></div><div class='line' id='LC31'><span class="cp">#define IMU_ACC_DRDY_GPIO_PORTSOURCE  GPIO_PortSourceGPIOB</span></div><div class='line' id='LC32'><br/></div><div class='line' id='LC33'><span class="cp">/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/</span></div><div class='line' id='LC34'><span class="cp">#ifndef ADC_CHANNEL_VSUPPLY</span></div><div class='line' id='LC35'><span class="cp">#define ADC_CHANNEL_VSUPPLY 2</span></div><div class='line' id='LC36'><span class="cp">#endif</span></div><div class='line' id='LC37'><span class="cp">#define DefaultVoltageOfAdc(adc) (0.00485*adc)</span></div><div class='line' id='LC38'><br/></div><div class='line' id='LC39'><span class="cm">/* Onboard ADCs */</span></div><div class='line' id='LC40'><span class="cm">/*</span></div><div class='line' id='LC41'><span class="cm">   ADC1 PC3/ADC13</span></div><div class='line' id='LC42'><span class="cm">   ADC2 PA0/ADC0</span></div><div class='line' id='LC43'><span class="cm">   ADC3 PC0/ADC10</span></div><div class='line' id='LC44'><span class="cm">   ADC4 PC1/ADC11</span></div><div class='line' id='LC45'><span class="cm">   ADC5 PC5/ADC15</span></div><div class='line' id='LC46'><span class="cm">   ADC6 PA1/ADC1</span></div><div class='line' id='LC47'><span class="cm">   ADC7 PC2/ADC12</span></div><div class='line' id='LC48'><span class="cm">   BATT PC4/ADC14</span></div><div class='line' id='LC49'><span class="cm">*/</span></div><div class='line' id='LC50'><span class="cp">#define BOARD_ADC_CHANNEL_1 ADC_Channel_13</span></div><div class='line' id='LC51'><span class="cp">#define BOARD_ADC_CHANNEL_2 ADC_Channel_0</span></div><div class='line' id='LC52'><span class="cp">// FIXME - removed for now and used for battery monitoring</span></div><div class='line' id='LC53'><span class="cp">//#define BOARD_ADC_CHANNEL_3 ADC_Channel_10</span></div><div class='line' id='LC54'><span class="cp">#define BOARD_ADC_CHANNEL_3 ADC_Channel_14</span></div><div class='line' id='LC55'><span class="cp">#define BOARD_ADC_CHANNEL_4 ADC_Channel_11</span></div><div class='line' id='LC56'><br/></div><div class='line' id='LC57'><span class="cp">#define BOARD_HAS_BARO</span></div><div class='line' id='LC58'><br/></div><div class='line' id='LC59'><span class="cp">#define USE_OPENCM3</span></div><div class='line' id='LC60'><br/></div><div class='line' id='LC61'><span class="cp">#define HSE_TYPE_EXT_CLK</span></div><div class='line' id='LC62'><span class="cp">#define STM32_RCC_MODE RCC_HSE_ON</span></div><div class='line' id='LC63'><span class="cp">#define STM32_PLL_MULT RCC_PLLMul_6</span></div><div class='line' id='LC64'><br/></div><div class='line' id='LC65'><span class="cp">#define PWM_5AND6_TIMER TIM5</span></div><div class='line' id='LC66'><span class="cp">#define PWM_5AND6_RCC RCC_APB1Periph_TIM5</span></div><div class='line' id='LC67'><span class="cp">#define PWM5_OC 1</span></div><div class='line' id='LC68'><span class="cp">#define PWM6_OC 2</span></div><div class='line' id='LC69'><span class="cp">#define PWM_5AND6_GPIO GPIOA</span></div><div class='line' id='LC70'><span class="cp">#define PWM5_Pin GPIO_Pin_0</span></div><div class='line' id='LC71'><span class="cp">#define PWM6_Pin GPIO_Pin_1</span></div><div class='line' id='LC72'><br/></div><div class='line' id='LC73'><span class="cp">#endif </span><span class="cm">/* CONFIG_LISA_M_1_0_H */</span><span class="cp"></span></div></pre></div>
          </td>
        </tr>
      </table>
  </div>

          </div>
        </div>
      </div>
    </div>

  </div>

<div class="frame frame-loading" style="display:none;" data-tree-list-url="/paparazzi/paparazzi/tree-list/0ed489b9d13c4476517d632660db4e31e1b6e92b" data-blob-url-prefix="/paparazzi/paparazzi/blob/0ed489b9d13c4476517d632660db4e31e1b6e92b">
  <img src="https://a248.e.akamai.net/assets.github.com/images/modules/ajax/big_spinner_336699.gif?1310086001" height="32" width="32">
</div>

      </div>
      <div class="context-overlay"></div>
    </div>


      <!-- footer -->
      <div id="footer" >
        
  <div class="upper_footer">
     <div class="container clearfix">

       <!--[if IE]><h4 id="blacktocat_ie">GitHub Links</h4><![endif]-->
       <![if !IE]><h4 id="blacktocat">GitHub Links</h4><![endif]>

       <ul class="footer_nav">
         <h4>GitHub</h4>
         <li><a href="https://github.com/about">About</a></li>
         <li><a href="https://github.com/blog">Blog</a></li>
         <li><a href="https://github.com/features">Features</a></li>
         <li><a href="https://github.com/contact">Contact &amp; Support</a></li>
         <li><a href="https://github.com/training">Training</a></li>
         <li><a href="http://enterprise.github.com/">GitHub Enterprise</a></li>
         <li><a href="http://status.github.com/">Site Status</a></li>
       </ul>

       <ul class="footer_nav">
         <h4>Tools</h4>
         <li><a href="http://get.gaug.es/">Gauges: Analyze web traffic</a></li>
         <li><a href="http://speakerdeck.com">Speaker Deck: Presentations</a></li>
         <li><a href="https://gist.github.com">Gist: Code snippets</a></li>
         <li><a href="http://mac.github.com/">GitHub for Mac</a></li>
         <li><a href="http://mobile.github.com/">Issues for iPhone</a></li>
         <li><a href="http://jobs.github.com/">Job Board</a></li>
       </ul>

       <ul class="footer_nav">
         <h4>Extras</h4>
         <li><a href="http://shop.github.com/">GitHub Shop</a></li>
         <li><a href="http://octodex.github.com/">The Octodex</a></li>
       </ul>

       <ul class="footer_nav">
         <h4>Documentation</h4>
         <li><a href="http://help.github.com/">GitHub Help</a></li>
         <li><a href="http://developer.github.com/">Developer API</a></li>
         <li><a href="http://github.github.com/github-flavored-markdown/">GitHub Flavored Markdown</a></li>
         <li><a href="http://pages.github.com/">GitHub Pages</a></li>
       </ul>

     </div><!-- /.site -->
  </div><!-- /.upper_footer -->

<div class="lower_footer">
  <div class="container clearfix">
    <!--[if IE]><div id="legal_ie"><![endif]-->
    <![if !IE]><div id="legal"><![endif]>
      <ul>
          <li><a href="https://github.com/site/terms">Terms of Service</a></li>
          <li><a href="https://github.com/site/privacy">Privacy</a></li>
          <li><a href="https://github.com/security">Security</a></li>
      </ul>

      <p>&copy; 2012 <span id="_rrt" title="0.07723s from fe7.rs.github.com">GitHub</span> Inc. All rights reserved.</p>
    </div><!-- /#legal or /#legal_ie-->

      <div class="sponsor">
        <a href="http://www.rackspace.com" class="logo">
          <img alt="Dedicated Server" height="36" src="https://a248.e.akamai.net/assets.github.com/images/modules/footer/rackspace_logo.png?v2" width="38" />
        </a>
        Powered by the <a href="http://www.rackspace.com ">Dedicated
        Servers</a> and<br/> <a href="http://www.rackspacecloud.com">Cloud
        Computing</a> of Rackspace Hosting<span>&reg;</span>
      </div>
  </div><!-- /.site -->
</div><!-- /.lower_footer -->

      </div><!-- /#footer -->

    

<div id="keyboard_shortcuts_pane" class="instapaper_ignore readability-extra" style="display:none">
  <h2>Keyboard Shortcuts <small><a href="#" class="js-see-all-keyboard-shortcuts">(see all)</a></small></h2>

  <div class="columns threecols">
    <div class="column first">
      <h3>Site wide shortcuts</h3>
      <dl class="keyboard-mappings">
        <dt>s</dt>
        <dd>Focus site search</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>?</dt>
        <dd>Bring up this help dialog</dd>
      </dl>
    </div><!-- /.column.first -->

    <div class="column middle" style='display:none'>
      <h3>Commit list</h3>
      <dl class="keyboard-mappings">
        <dt>j</dt>
        <dd>Move selection down</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>k</dt>
        <dd>Move selection up</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>c <em>or</em> o <em>or</em> enter</dt>
        <dd>Open commit</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>y</dt>
        <dd>Expand URL to its canonical form</dd>
      </dl>
    </div><!-- /.column.first -->

    <div class="column last" style='display:none'>
      <h3>Pull request list</h3>
      <dl class="keyboard-mappings">
        <dt>j</dt>
        <dd>Move selection down</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>k</dt>
        <dd>Move selection up</dd>
      </dl>
      <dl class="keyboard-mappings">
        <dt>o <em>or</em> enter</dt>
        <dd>Open issue</dd>
      </dl>
    </div><!-- /.columns.last -->

  </div><!-- /.columns.equacols -->

  <div style='display:none'>
    <div class="rule"></div>

    <h3>Issues</h3>

    <div class="columns threecols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt>j</dt>
          <dd>Move selection down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>k</dt>
          <dd>Move selection up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>x</dt>
          <dd>Toggle selection</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o <em>or</em> enter</dt>
          <dd>Open issue</dd>
        </dl>
      </div><!-- /.column.first -->
      <div class="column middle">
        <dl class="keyboard-mappings">
          <dt>I</dt>
          <dd>Mark selection as read</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>U</dt>
          <dd>Mark selection as unread</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>e</dt>
          <dd>Close selection</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>y</dt>
          <dd>Remove selection from view</dd>
        </dl>
      </div><!-- /.column.middle -->
      <div class="column last">
        <dl class="keyboard-mappings">
          <dt>c</dt>
          <dd>Create issue</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>l</dt>
          <dd>Create label</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>i</dt>
          <dd>Back to inbox</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>u</dt>
          <dd>Back to issues</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>/</dt>
          <dd>Focus issues search</dd>
        </dl>
      </div>
    </div>
  </div>

  <div style='display:none'>
    <div class="rule"></div>

    <h3>Issues Dashboard</h3>

    <div class="columns threecols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt>j</dt>
          <dd>Move selection down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>k</dt>
          <dd>Move selection up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>o <em>or</em> enter</dt>
          <dd>Open issue</dd>
        </dl>
      </div><!-- /.column.first -->
    </div>
  </div>

  <div style='display:none'>
    <div class="rule"></div>

    <h3>Network Graph</h3>
    <div class="columns equacols">
      <div class="column first">
        <dl class="keyboard-mappings">
          <dt><span class="badmono">←</span> <em>or</em> h</dt>
          <dd>Scroll left</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">→</span> <em>or</em> l</dt>
          <dd>Scroll right</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">↑</span> <em>or</em> k</dt>
          <dd>Scroll up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt><span class="badmono">↓</span> <em>or</em> j</dt>
          <dd>Scroll down</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>t</dt>
          <dd>Toggle visibility of head labels</dd>
        </dl>
      </div><!-- /.column.first -->
      <div class="column last">
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">←</span> <em>or</em> shift h</dt>
          <dd>Scroll all the way left</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">→</span> <em>or</em> shift l</dt>
          <dd>Scroll all the way right</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">↑</span> <em>or</em> shift k</dt>
          <dd>Scroll all the way up</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>shift <span class="badmono">↓</span> <em>or</em> shift j</dt>
          <dd>Scroll all the way down</dd>
        </dl>
      </div><!-- /.column.last -->
    </div>
  </div>

  <div >
    <div class="rule"></div>
    <div class="columns threecols">
      <div class="column first" >
        <h3>Source Code Browsing</h3>
        <dl class="keyboard-mappings">
          <dt>t</dt>
          <dd>Activates the file finder</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>l</dt>
          <dd>Jump to line</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>w</dt>
          <dd>Switch branch/tag</dd>
        </dl>
        <dl class="keyboard-mappings">
          <dt>y</dt>
          <dd>Expand URL to its canonical form</dd>
        </dl>
      </div>
    </div>
  </div>
</div>

    <div id="markdown-help" class="instapaper_ignore readability-extra">
  <h2>Markdown Cheat Sheet</h2>

  <div class="cheatsheet-content">

  <div class="mod">
    <div class="col">
      <h3>Format Text</h3>
      <p>Headers</p>
      <pre>
# This is an &lt;h1&gt; tag
## This is an &lt;h2&gt; tag
###### This is an &lt;h6&gt; tag</pre>
     <p>Text styles</p>
     <pre>
*This text will be italic*
_This will also be italic_
**This text will be bold**
__This will also be bold__

*You **can** combine them*
</pre>
    </div>
    <div class="col">
      <h3>Lists</h3>
      <p>Unordered</p>
      <pre>
* Item 1
* Item 2
  * Item 2a
  * Item 2b</pre>
     <p>Ordered</p>
     <pre>
1. Item 1
2. Item 2
3. Item 3
   * Item 3a
   * Item 3b</pre>
    </div>
    <div class="col">
      <h3>Miscellaneous</h3>
      <p>Images</p>
      <pre>
![GitHub Logo](/images/logo.png)
Format: ![Alt Text](url)
</pre>
     <p>Links</p>
     <pre>
http://github.com - automatic!
[GitHub](http://github.com)</pre>
<p>Blockquotes</p>
     <pre>
As Kanye West said:

> We're living the future so
> the present is our past.
</pre>
    </div>
  </div>
  <div class="rule"></div>

  <h3>Code Examples in Markdown</h3>
  <div class="col">
      <p>Syntax highlighting with <a href="http://github.github.com/github-flavored-markdown/" title="GitHub Flavored Markdown" target="_blank">GFM</a></p>
      <pre>
```javascript
function fancyAlert(arg) {
  if(arg) {
    $.facebox({div:'#foo'})
  }
}
```</pre>
    </div>
    <div class="col">
      <p>Or, indent your code 4 spaces</p>
      <pre>
Here is a Python code example
without syntax highlighting:

    def foo:
      if not bar:
        return true</pre>
    </div>
    <div class="col">
      <p>Inline code for comments</p>
      <pre>
I think you should use an
`&lt;addr&gt;` element here instead.</pre>
    </div>
  </div>

  </div>
</div>


    <div class="ajax-error-message">
      <p><span class="icon"></span> Something went wrong with that request. Please try again. <a href="javascript:;" class="ajax-error-dismiss">Dismiss</a></p>
    </div>

    
    
    
  </body>
</html>

