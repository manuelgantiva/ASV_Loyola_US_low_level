// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "asv_interfaces/msg/detail/pwm_values__struct.h"
#include "asv_interfaces/msg/detail/pwm_values__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool asv_interfaces__msg__pwm_values__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("asv_interfaces.msg._pwm_values.PwmValues", full_classname_dest, 40) == 0);
  }
  asv_interfaces__msg__PwmValues * ros_message = _ros_message;
  {  // t_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_left");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t_left = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // t_righ
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_righ");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->t_righ = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * asv_interfaces__msg__pwm_values__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PwmValues */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("asv_interfaces.msg._pwm_values");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PwmValues");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  asv_interfaces__msg__PwmValues * ros_message = (asv_interfaces__msg__PwmValues *)raw_ros_message;
  {  // t_left
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->t_left);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_righ
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->t_righ);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_righ", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
