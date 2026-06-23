#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to asv_interfaces__msg__PwmValues
/// This is a message with pwm values
/// If you want to embed it in another message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PwmValues::default())
  }
}

impl rosidl_runtime_rs::Message for PwmValues {
  type RmwMsg = super::msg::rmw::PwmValues;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        t_left: msg.t_left,
        t_righ: msg.t_righ,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      t_left: msg.t_left,
      t_righ: msg.t_righ,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      t_left: msg.t_left,
      t_righ: msg.t_righ,
    }
  }
}


// Corresponds to asv_interfaces__msg__StateNeighbor
/// this is a message used to communicate the results of the state observer Neighbor
/// If you want to embed it in another message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StateNeighbor {

    // This member is not documented.
    #[allow(missing_docs)]
    pub id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub point: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub msg_from: i64,

}



impl Default for StateNeighbor {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StateNeighbor::default())
  }
}

impl rosidl_runtime_rs::Message for StateNeighbor {
  type RmwMsg = super::msg::rmw::StateNeighbor;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id.as_str().into(),
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.point)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        msg_from: msg.msg_from,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id.as_str().into(),
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.point)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
      msg_from: msg.msg_from,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id.to_string(),
      point: geometry_msgs::msg::Point::from_rmw_message(msg.point),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      msg_from: msg.msg_from,
    }
  }
}


// Corresponds to asv_interfaces__msg__StateObserver
/// this is a message used to communicate the results of the state observer
/// If you want to embed it in another message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StateObserver {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub point: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub disturbances: geometry_msgs::msg::Vector3,

}



impl Default for StateObserver {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StateObserver::default())
  }
}

impl rosidl_runtime_rs::Message for StateObserver {
  type RmwMsg = super::msg::rmw::StateObserver;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.point)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        disturbances: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.disturbances)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        point: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.point)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        disturbances: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.disturbances)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      point: geometry_msgs::msg::Point::from_rmw_message(msg.point),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      disturbances: geometry_msgs::msg::Vector3::from_rmw_message(msg.disturbances),
    }
  }
}


// Corresponds to asv_interfaces__msg__XbeeObserver
/// this is a message used to communicate the results of the state observer by Xbee
/// If you want to embed it in another message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct XbeeObserver {

    // This member is not documented.
    #[allow(missing_docs)]
    pub counter: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub states: Vec<super::msg::StateNeighbor>,

}



impl Default for XbeeObserver {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::XbeeObserver::default())
  }
}

impl rosidl_runtime_rs::Message for XbeeObserver {
  type RmwMsg = super::msg::rmw::XbeeObserver;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        counter: msg.counter,
        states: msg.states
          .into_iter()
          .map(|elem| super::msg::StateNeighbor::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      counter: msg.counter,
        states: msg.states
          .iter()
          .map(|elem| super::msg::StateNeighbor::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      counter: msg.counter,
      states: msg.states
          .into_iter()
          .map(super::msg::StateNeighbor::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to asv_interfaces__msg__ReferenceLlc
/// This is a message with References velocities values
/// If you want to embed it in another message.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReferenceLlc {

    // This member is not documented.
    #[allow(missing_docs)]
    pub references: Vec<geometry_msgs::msg::Vector3>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub u_tar: std_msgs::msg::Float64,

}



impl Default for ReferenceLlc {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ReferenceLlc::default())
  }
}

impl rosidl_runtime_rs::Message for ReferenceLlc {
  type RmwMsg = super::msg::rmw::ReferenceLlc;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        references: msg.references
          .into_iter()
          .map(|elem| geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
        u_tar: std_msgs::msg::Float64::into_rmw_message(std::borrow::Cow::Owned(msg.u_tar)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        references: msg.references
          .iter()
          .map(|elem| geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
        u_tar: std_msgs::msg::Float64::into_rmw_message(std::borrow::Cow::Borrowed(&msg.u_tar)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      references: msg.references
          .into_iter()
          .map(geometry_msgs::msg::Vector3::from_rmw_message)
          .collect(),
      u_tar: std_msgs::msg::Float64::from_rmw_message(msg.u_tar),
    }
  }
}


