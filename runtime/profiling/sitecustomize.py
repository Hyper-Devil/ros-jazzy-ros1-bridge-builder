"""Enable embedded EMCP instrumentation only for the explicit benchmark environment."""
import os
if os.environ.get('UAM2TE_PROFILE_DIR'):
    import importlib.abc
    import importlib.machinery
    import sys
    class ProfileFinder(importlib.abc.MetaPathFinder):
        def find_spec(self,fullname,path=None,target=None):
            if fullname!='elevation_mapping_cupy.elevation_mapping':return None
            spec=importlib.machinery.PathFinder.find_spec(fullname,path)
            if spec is None:return None
            original=spec.loader.exec_module
            def execute(module):
                original(module)
                from runtime_profile import instrument_elevation
                instrument_elevation(module)
            spec.loader.exec_module=execute
            return spec
    sys.meta_path.insert(0,ProfileFinder())
