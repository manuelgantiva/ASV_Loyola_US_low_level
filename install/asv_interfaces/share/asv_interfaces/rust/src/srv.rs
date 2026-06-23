#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to asv_interfaces__srv__SetObs_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetObs_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetObs_Request {
  type RmwMsg = super::srv::rmw::SetObs_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        eso_mode: msg.eso_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      eso_mode: msg.eso_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      eso_mode: msg.eso_mode,
    }
  }
}


// Corresponds to asv_interfaces__srv__SetObs_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetObs_Response {
    /// Mode correctly and SET_ESO are sent
    pub success: bool,

}



impl Default for SetObs_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetObs_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetObs_Response {
  type RmwMsg = super::srv::rmw::SetObs_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
  }
}


// Corresponds to asv_interfaces__srv__SetLlc_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetLlc_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetLlc_Request {
  type RmwMsg = super::srv::rmw::SetLlc_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        llc_mode: msg.llc_mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      llc_mode: msg.llc_mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      llc_mode: msg.llc_mode,
    }
  }
}


// Corresponds to asv_interfaces__srv__SetLlc_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetLlc_Response {
    /// Mode correctly and SET_ESO are sent
    pub success: bool,

}



impl Default for SetLlc_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetLlc_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetLlc_Response {
  type RmwMsg = super::srv::rmw::SetLlc_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
    }
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


