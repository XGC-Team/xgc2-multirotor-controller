"""Packaging diagnostics without compiling, installing or creating a Debian package."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT=Path(__file__).with_name('package_debs.sh')


class PackagePrerequisiteTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        binary=self.root/'bin'; binary.mkdir()
        self.file=binary/'file'
        self.env=dict(os.environ,PATH=str(binary)+':'+os.environ['PATH'],PACKAGE_VERSION='1.0',ROS_DISTRO='noetic')
        modules=self.root/'install/opt/ros/noetic/lib'; modules.mkdir(parents=True)
        for name in ('reference','controller'):
            (modules/f'libpx4_multirotor_{name}_module.so').write_text('fixture')

    def invoke(self):
        return subprocess.run(['bash',str(SCRIPT),'--install-root',str(self.root/'install'),
                               '--output-dir',str(self.root/'out')],env=self.env,text=True,capture_output=True,timeout=5)

    def stub_file(self,body):
        self.file.write_text('#!/bin/bash\n'+body+'\n'); self.file.chmod(0o755)

    def test_missing_tool_is_not_reported_as_non_elf(self):
        wrapper=self.root/'bash-env'
        wrapper.write_text('command() { if [[ "$*" == "-v file" ]]; then return 1; fi; builtin command "$@"; }\n')
        self.env['BASH_ENV']=str(wrapper)
        result=self.invoke()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('missing packaging tool: file',result.stderr)
        self.assertNotIn('not ELF',result.stderr)
        self.assertFalse((self.root/'out').exists())

    def test_broken_tool_has_a_distinct_diagnostic(self):
        self.stub_file('exit 127')
        result=self.invoke()
        self.assertIn('packaging tool is not usable: file',result.stderr)
        self.assertNotIn('not ELF',result.stderr)

    def test_inspection_failure_is_not_reported_as_non_elf(self):
        self.stub_file('[[ "$1" == --version ]] && exit 0\nexit 2')
        result=self.invoke()
        self.assertIn('failed to inspect module library with file:',result.stderr)
        self.assertNotIn('not ELF',result.stderr)

    def test_real_non_elf_check_is_preserved(self):
        self.stub_file('echo "ASCII text"')
        result=self.invoke()
        self.assertNotEqual(result.returncode,0)
        self.assertIn('required module library is not ELF:',result.stderr)
        self.assertFalse((self.root/'out').exists())


if __name__=='__main__':
    unittest.main()
