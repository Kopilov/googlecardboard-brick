Android GoogleVR and OpenGL learning application
=======================

Based on [Google Cardboard](https://github.com/googlevr/cardboard) demo, [learnopengl tutorials](https://learnopengl.com)
and previously written project https://github.com/Kopilov/opengl-brick.

As opposed to [original](https://github.com/googlevr/cardboard), this project keeps
Cardboard SKD framework in a separate Git submodule.

Also, [Assimp](https://github.com/assimp/assimp) and [GLM](https://github.com/g-truc/glm)
GitHub repositories are used as submodules dependencies.

After cloning this project, run `git submodule update --init`, then build and execute it
following [original guideline](https://developers.google.com/cardboard/develop/c/quickstart#download_and_build_the_demo_app).

Your device should support OpenGL ES 3.0 to run the application.
IRL, tested on Nokia 7 plus

