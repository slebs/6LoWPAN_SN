<%@page import="serial.SerialComm"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");
    if (benutzerName == null) {
        response.sendRedirect("login.jsp");
        session.setAttribute("error", "0");
        return;
    }

    // TODO: Implement the node addressing (1)
    SerialComm sc = (SerialComm) session.getAttribute("sc");
    if (request.getParameter("getdata") != null) {

        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETDATA, 1);
        // SerialComm.getInstance().disconnect();
    } else if (request.getParameter("getecho") != null) {
        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETECHO, 1);
    } else if (request.getParameter("getnodes") != null) {
        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETNODES, 1);
    }else if(request.getParameter("disconnect")!=null){
        SerialComm.getInstance().disconnect();
    }
    response.sendRedirect("content.jsp");
%>