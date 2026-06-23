#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__PwmValues() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__msg__PwmValues__init(msg: *mut PwmValues) -> bool;
    fn asv_interfaces__msg__PwmValues__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PwmValues>, size: usize) -> bool;
    fn asv_interfaces__msg__PwmValues__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PwmValues>);
    fn asv_interfaces__msg__PwmValues__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PwmValues>, out_seq: *mut rosidl_runtime_rs::Sequence<PwmValues>) -> bool;
}

// Corresponds to asv_interfaces__msg__PwmValues
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This is a message with pwm values
/// If you want to embed it in another message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PwmValues {

    // This member is not documented.
    #[allow(missing_docs)]
    pub t_left: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub t_righ: u16,

}



impl Default for PwmValues {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__msg__PwmValues__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__msg__PwmValues__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PwmValues {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__PwmValues__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__PwmValues__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__PwmValues__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PwmValues {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PwmValues where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/msg/PwmValues";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__PwmValues() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__StateNeighbor() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__msg__StateNeighbor__init(msg: *mut StateNeighbor) -> bool;
    fn asv_interfaces__msg__StateNeighbor__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StateNeighbor>, size: usize) -> bool;
    fn asv_interfaces__msg__StateNeighbor__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StateNeighbor>);
    fn asv_interfaces__msg__StateNeighbor__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StateNeighbor>, out_seq: *mut rosidl_runtime_rs::Sequence<StateNeighbor>) -> bool;
}

// Corresponds to asv_interfaces__msg__StateNeighbor
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// this is a message used to communicate the results of the state observer Neighbor
/// If you want to embed it in another message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StateNeighbor {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub point: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub msg_from: i64,

}



impl Default for StateNeighbor {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__msg__StateNeighbor__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__msg__StateNeighbor__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StateNeighbor {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateNeighbor__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateNeighbor__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateNeighbor__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StateNeighbor {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StateNeighbor where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/msg/StateNeighbor";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__StateNeighbor() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__StateObserver() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__msg__StateObserver__init(msg: *mut StateObserver) -> bool;
    fn asv_interfaces__msg__StateObserver__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StateObserver>, size: usize) -> bool;
    fn asv_interfaces__msg__StateObserver__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StateObserver>);
    fn asv_interfaces__msg__StateObserver__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StateObserver>, out_seq: *mut rosidl_runtime_rs::Sequence<StateObserver>) -> bool;
}

// Corresponds to asv_interfaces__msg__StateObserver
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// this is a message used to communicate the results of the state observer
/// If you want to embed it in another message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StateObserver {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub point: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub disturbances: geometry_msgs::msg::rmw::Vector3,

}



impl Default for StateObserver {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__msg__StateObserver__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__msg__StateObserver__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StateObserver {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateObserver__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateObserver__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__StateObserver__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StateObserver {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StateObserver where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/msg/StateObserver";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__StateObserver() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__XbeeObserver() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__msg__XbeeObserver__init(msg: *mut XbeeObserver) -> bool;
    fn asv_interfaces__msg__XbeeObserver__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<XbeeObserver>, size: usize) -> bool;
    fn asv_interfaces__msg__XbeeObserver__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<XbeeObserver>);
    fn asv_interfaces__msg__XbeeObserver__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<XbeeObserver>, out_seq: *mut rosidl_runtime_rs::Sequence<XbeeObserver>) -> bool;
}

// Corresponds to asv_interfaces__msg__XbeeObserver
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// this is a message used to communicate the results of the state observer by Xbee
/// If you want to embed it in another message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct XbeeObserver {

    // This member is not documented.
    #[allow(missing_docs)]
    pub counter: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub states: rosidl_runtime_rs::Sequence<super::super::msg::rmw::StateNeighbor>,

}



impl Default for XbeeObserver {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__msg__XbeeObserver__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__msg__XbeeObserver__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for XbeeObserver {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__XbeeObserver__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__XbeeObserver__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__XbeeObserver__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for XbeeObserver {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for XbeeObserver where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/msg/XbeeObserver";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__XbeeObserver() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__ReferenceLlc() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__msg__ReferenceLlc__init(msg: *mut ReferenceLlc) -> bool;
    fn asv_interfaces__msg__ReferenceLlc__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReferenceLlc>, size: usize) -> bool;
    fn asv_interfaces__msg__ReferenceLlc__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReferenceLlc>);
    fn asv_interfaces__msg__ReferenceLlc__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReferenceLlc>, out_seq: *mut rosidl_runtime_rs::Sequence<ReferenceLlc>) -> bool;
}

// Corresponds to asv_interfaces__msg__ReferenceLlc
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// This is a message with References velocities values
/// If you want to embed it in another message.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReferenceLlc {

    // This member is not documented.
    #[allow(missing_docs)]
    pub references: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Vector3>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub u_tar: std_msgs::msg::rmw::Float64,

}



impl Default for ReferenceLlc {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__msg__ReferenceLlc__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__msg__ReferenceLlc__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReferenceLlc {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__ReferenceLlc__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__ReferenceLlc__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__msg__ReferenceLlc__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReferenceLlc {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReferenceLlc where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/msg/ReferenceLlc";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__msg__ReferenceLlc() }
  }
}


