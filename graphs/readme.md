## Tex graph generation 


To generate the graphs from the dot files, you can use the `dot2tex` tool :
```bash
dot2tex local_planner.dot -o local_planner.tex --figonly
```


`dot2tex` can be installed on Ubuntu with the following command:
```bash
sudo apt-get install dot2tex
```