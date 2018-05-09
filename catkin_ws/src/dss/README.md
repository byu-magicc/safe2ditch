Ditch Site Selection ROS Wrapper
================================

This package is purely to expose the ditch site selector Python package from [nasa/Safe2Ditch](https://github.com/nasa/Safe2Ditch) into the ROS workspace. The source code remains in NASA's [GitHub repo](https://github.com/nasa/Safe2Ditch) and is included as a submodule here, as the `src` directory.

To see how the `dss` package is used, investigate the `nasa_s2d/scripts/dss_node.py` script, which creates a ROS interface and starts a DSS manager (`dss.core.manager`).