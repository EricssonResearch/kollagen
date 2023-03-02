#include <Python.h>
#include "kollagen/generate.h"

static PyObject* generate_python(PyObject * /*unused*/, PyObject* args){
  const char * _json_file = nullptr;
  const char * _save_dir = nullptr;
  int _multig2o = 0;
  int _singleg2o = 0;
  if (!PyArg_ParseTuple(args, "ssii", &_json_file, &_save_dir, &_multig2o, &_singleg2o)) { return NULL; }

  std::filesystem::path json_file{_json_file};
  std::filesystem::path save_dir{_save_dir};
  bool multig2o{static_cast<bool>(_multig2o)};
  bool singleg2o{static_cast<bool>(_singleg2o)};

  kollagen::add_extension(json_file, "json");

  if (!kollagen::CLI_arguments_OK(json_file, save_dir, multig2o, singleg2o)) {
      PyErr_SetString(PyExc_Exception, "Improper arguments provided");
      return NULL;
  }

  kollagen::generate_and_save(json_file, save_dir, multig2o, singleg2o);

  Py_RETURN_NONE;
}

static PyMethodDef kollagen_methods[] = {
    {"generate", (PyCFunction)generate_python, METH_VARARGS, nullptr},
    {nullptr, nullptr, 0, nullptr}
};

static struct PyModuleDef kollagen_module = {
    PyModuleDef_HEAD_INIT, "kollagen", NULL, -1, kollagen_methods
};

PyMODINIT_FUNC PyInit_kollagen(void) {
  return PyModule_Create(&kollagen_module);
}
