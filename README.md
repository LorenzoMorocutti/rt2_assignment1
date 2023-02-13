# How to run

In this branch there is the implementation of the jupyter notebook, that substitutes the old user_interface.py script and provides a more friendly-user interface.

To launch the simulation, please write in the terminal:
```
roslaunch rt2_assignment1 sim.launch
```

After that, open a new tab and go to the rt2_assignment1 folder and write:
```
jupyter notebook --allow-root
```

This will open a browser page, click on the ui_notebook.jpynb (our notebook) and then click on the 'run >' button (in the upper part of the window) for each block. It's reccomended to run each block separately because some variables are declared at the beginning of the application and we don't want to mix the launch order of the blocks. 

## Documentation

The documentation of the code (scripts + cpp files) is in the *Doxygen* formalism.
There is also a *Sphinx* branch containing the documentation in the Sphinx formalism.