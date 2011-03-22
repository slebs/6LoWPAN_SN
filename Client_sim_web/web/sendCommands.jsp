<%@page import="serial.SerialComm"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");
    if (benutzerName == null) {
        response.sendRedirect("login.jsp");
        session.setAttribute("error", "0");
        return;
    }

    if (request.getParameter("getdata") != null) {

        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETDATA, Integer.parseInt(request.getParameter("node")));
        // SerialComm.getInstance().disconnect();
    } else if (request.getParameter("getecho") != null) {
        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETECHO, Integer.parseInt(request.getParameter("node")));
    } else if (request.getParameter("getnodes") != null) {
        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETNODES);
    } else if (request.getParameter("disconnect") != null) {
        SerialComm.getInstance().disconnect();
    } else if (request.getParameter("connect") != null) {
        SerialComm.getInstance().connect(request.getParameter("port"), Integer.parseInt(request.getParameter("baud")));
    } else if (request.getParameter("getnodeaddress") != null) {
        SerialComm.getInstance().sendCommand(SerialComm.COMMAND_GETCONNECTEDNODE);
    }
    response.sendRedirect("content.jsp");
%>