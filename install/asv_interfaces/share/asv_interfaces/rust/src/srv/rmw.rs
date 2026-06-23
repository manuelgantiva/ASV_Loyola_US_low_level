#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetObs_Request() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__srv__SetObs_Request__init(msg: *mut SetObs_Request) -> bool;
    fn asv_interfaces__srv__SetObs_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetObs_Request>, size: usize) -> bool;
    fn asv_interfaces__srv__SetObs_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetObs_Request>);
    fn asv_interfaces__srv__SetObs_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetObs_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetObs_Request>) -> bool;
}

// Corresponds to asv_interfaces__srv__SetObs_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetObs_Request {
    /// filled by ESO_MODE enum value
    pub eso_mode: u8,

}

impl SetObs_Request {
    /// basic ESO_MODE
    pub const ESO_BEJARANO: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ESO_LIU: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ESO_ZONO: u8 = 3;

}


impl Default for SetObs_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__srv__SetObs_Request__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__srv__SetObs_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetObs_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetObs_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetObs_Request where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/srv/SetObs_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetObs_Request() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetObs_Response() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__srv__SetObs_Response__init(msg: *mut SetObs_Response) -> bool;
    fn asv_interfaces__srv__SetObs_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetObs_Response>, size: usize) -> bool;
    fn asv_interfaces__srv__SetObs_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetObs_Response>);
    fn asv_interfaces__srv__SetObs_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetObs_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetObs_Response>) -> bool;
}

// Corresponds to asv_interfaces__srv__SetObs_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetObs_Response {
    /// Mode correctly and SET_ESO are sent
    pub success: bool,

}



impl Default for SetObs_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__srv__SetObs_Response__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__srv__SetObs_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetObs_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetObs_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetObs_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetObs_Response where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/srv/SetObs_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetObs_Response() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetLlc_Request() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__srv__SetLlc_Request__init(msg: *mut SetLlc_Request) -> bool;
    fn asv_interfaces__srv__SetLlc_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Request>, size: usize) -> bool;
    fn asv_interfaces__srv__SetLlc_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Request>);
    fn asv_interfaces__srv__SetLlc_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetLlc_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Request>) -> bool;
}

// Corresponds to asv_interfaces__srv__SetLlc_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLlc_Request {
    /// filled by LLC_MODE enum value
    pub llc_mode: u8,

}

impl SetLlc_Request {
    /// basic LLC_MODE
    pub const LLC_APM: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LLC_IFAC: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const LLC_MPC: u8 = 3;

}


impl Default for SetLlc_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__srv__SetLlc_Request__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__srv__SetLlc_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetLlc_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetLlc_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetLlc_Request where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/srv/SetLlc_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetLlc_Request() }
  }
}


#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetLlc_Response() -> *const std::ffi::c_void;
}

#[link(name = "asv_interfaces__rosidl_generator_c")]
extern "C" {
    fn asv_interfaces__srv__SetLlc_Response__init(msg: *mut SetLlc_Response) -> bool;
    fn asv_interfaces__srv__SetLlc_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Response>, size: usize) -> bool;
    fn asv_interfaces__srv__SetLlc_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Response>);
    fn asv_interfaces__srv__SetLlc_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetLlc_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetLlc_Response>) -> bool;
}

// Corresponds to asv_interfaces__srv__SetLlc_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLlc_Response {
    /// Mode correctly and SET_ESO are sent
    pub success: bool,

}



impl Default for SetLlc_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !asv_interfaces__srv__SetLlc_Response__init(&mut msg as *mut _) {
        panic!("Call to asv_interfaces__srv__SetLlc_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetLlc_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { asv_interfaces__srv__SetLlc_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetLlc_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetLlc_Response where Self: Sized {
  const TYPE_NAME: &'static str = "asv_interfaces/srv/SetLlc_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__asv_interfaces__srv__SetLlc_Response() }
  }
}






#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__asv_interfaces__srv__SetObs() -> *const std::ffi::c_void;
}

// Corresponds to asv_interfaces__srv__SetObs
#[allow(missing_docs, non_camel_case_types)]
pub struct SetObs;

impl rosidl_runtime_rs::Service for SetObs {
    type Request = SetObs_Request;
    type Response = SetObs_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__asv_interfaces__srv__SetObs() }
    }
}




#[link(name = "asv_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__asv_interfaces__srv__SetLlc() -> *const std::ffi::c_void;
}

// Corresponds to asv_interfaces__srv__SetLlc
#[allow(missing_docs, non_camel_case_types)]
pub struct SetLlc;

impl rosidl_runtime_rs::Service for SetLlc {
    type Request = SetLlc_Request;
    type Response = SetLlc_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__asv_interfaces__srv__SetLlc() }
    }
}


