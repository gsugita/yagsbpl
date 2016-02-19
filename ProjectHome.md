<font color='red'><b>Migration Notice</b> <i>(June, 2015)</i>:</font>

The new home for YAGSBPL on github is https://github.com/subh83/YAGSBPL . <br />This page on google code will be discontinued.


---



**Description:**

"Yet Another Graph-Search Based Planning Library (YAGSBPL)" is a fast, efficient and easy-to-use graph construction and search (using algorithms like A-star, Dijkstra's, etc.) library written in C++, designed specifically for searching medium to large scale graphs for optimal paths.

**Quick start:**
  * Download the latest library (v2.1): [yagsbpl-v2.1.zip](http://yagsbpl.googlecode.com/files/yagsbpl-v2.1.zip).
  * Download the basic example programs and follow instructions in README file: [yagsbpl-2.x\_basic-examples.zip](http://yagsbpl.googlecode.com/files/yagsbpl-2.x_basic-examples.zip).
  * Read the detailed documentation at http://goo.gl/q7WGE


**YAGSBPL is designed to be:**
  * _Fast_ (e.g., with integer coordinates for nodes but floating point cost as well as cost function needing to perform floating point operations online, and an average degree of the graph being 8, the library can expand about 70,000 nodes in the graph in just 1 second on a 2.1GHz processor machine with 2GB RAM.)
  * _Easy to use_ (being template-based, defining new arbitrary node-types, cost types, etc. is made easy. For graph connectivity, node accessibility tests, etc, pointers to user-defined functions can be used, which makes defining the graph structure very easy, yet highly flexible.)


**Basic code structure for the simplest usage:**
<table border='0' cellspacing='10px'><tr>
<td width='45%' align='center'><a href='http://www.subhrajit.net/files/Projects-IT/Cpp-Libraries/yagsbpl/yagsbpl1.png'><img src='http://www.subhrajit.net/files/Projects-IT/Cpp-Libraries/yagsbpl/yagsbpl1.png' width='80%' /></a></td>
<td width='45%' align='center'><a href='http://www.subhrajit.net/files/Projects-IT/Cpp-Libraries/yagsbpl/yagsbpl2.png'><img src='http://www.subhrajit.net/files/Projects-IT/Cpp-Libraries/yagsbpl/yagsbpl2.png' width='85%' /></a></td>
</tr><tr>
<td width='45%' align='center'><div><b>1.</b> A class describing the node/vertex type and</div><div>functions describing graph structure are defined</div><i></td></i><td width='45%' align='center'><div><b>2.</b> Initialization of graph, execution of search,</div><div>and reading the result done in the main function</div>_</td>
</tr></table>_


**For a detailed documentation and tutorial visit: http://goo.gl/q7WGE**
<br />


---


**YAGSBPL supports:**

- Directed graphs with complex cost functions.

- On the fly graph construction (i.e. no need to construct and store a complete graph before starting the search/planning process - makes it highly suitable for RRT-like graph construction).

- Arbitrary graph node type declaration.

- Arbitrary edge cost type declaration.

- Intermediate storage of paths during graph search.

- Multiple goals (or goal manifold) that determine when to stop search.

- Multiple seed nodes (start nodes) for wave-front expansion type graph exploration, and ability to trace seed lineage for any node.

- Event handling (i.e., call to user-defined functions upon generation, g-score updating or expansion of a node during the search process).

- Ability to write new planners with much ease. Comes with a weighted A-star (that includes Dijkstra's and normal A-star) planner by default.
<br />


---


If you found the library useful in research, please cite it as follows:
> _Subhrajit Bhattacharya (2013) "YAGSBPL: A template-based C++ library for large-scale graph search and planning". Available at http://subhrajit.net/index.php?WPage=yagsbpl_
Bibtex entry: <font size='1'>
<blockquote><div>@misc{yagsbpl,</div>
<div> title = {YAGSBPL: A template-based C++ library for large-scale graph search and planning},</div>
<div> author = {Subhrajit Bhattacharya},</div>
<div> note = {Available at <a href='http://subhrajit.net/index.php?WPage=yagsbpl'>http://subhrajit.net/index.php?WPage=yagsbpl</a> },</div>
<div> url = { <a href='http://subhrajit.net/index.php?WPage=yagsbpl'>http://subhrajit.net/index.php?WPage=yagsbpl</a> },</div>
<div> year = {2013}</div>
<div>}</div>
</font></blockquote>


---


<font color='green'>_<b>Upcoming release:</b></font> Planned release of v3.0 some time in mid/late 2013. Besides complete re-structuring and modularization of the code for greater flexibility and ease of development, there will be bug fixes, improved performance (with significant attention on code optimization), extensive support for multi-thread applications, and backward compatibility with v2.0. This will be accompanied by a more friendly documentation/tutorial._