# rros_template_repo
the template repository of RROS lab

Recommened prerequist: [create a new ROS package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)

## Structure
    .  
    ├── .gitignore  
    ├── .rosinstall  
    ├── action  
    │   └── actionRROS.action  
    ├── CMakeLists.txt  
    ├── data  
    │   └── test_data1.csv  
    ├── include  
    │   └── rros_template_repo  
    │       └── rros.hpp  
    ├── launch  
    │   └── rros.launch  
    ├── LICENSE  
    ├── msg  
    │   └── MessageRROS.msg  
    ├── package.xml  
    ├── README.md  
    ├── scripts  
    │   └── rros.py  
    ├── src  
    │   ├── demo  
    │   │   └── rros_demo.cpp  
    │   ├── rros.cpp  
    │   ├── test  
    │   │   └── unit_test.cpp  
    │   └── utilities  
    │       └── rros_utility.cpp  
    └── srv  
        └── ServiceRROS.srv  

### Required directories and files
- include (dir)
- src (dir)
- CMakeLists.txt  
- package.xml 
- .gitignore
- .rosinstall (required when the repo is public)
- LICENSE
- README.md

### Recommended directories
- launch 
- src/demo 
- src/test 
- src/utilities 

### The "data" directory
- data folder is only used during development, remove it when you make the repo public
- do not upload files larger than 10MB


## Change Package Name
If you want to change the package name, here are what you should change:  

- line 2 in CMakeLists.txt  
  `project([new package name])`
- line 3 in package.xml  
  `<name>[new package name]</name>`
- current directory name
- directory name under include folder  
  `include/[new package name]`

or just replace all `rros_template_repo` with `[new package name]`