<%-- 
    Document   : content.jsp
    Created on : Dec 16, 2010, 3:49:29 PM
    Author     : simon
--%>
<%@page import="serial.SerialComm"%>
<%@page import="java.util.Map.Entry"%>
<%@page import="jdbc.DBManager"%>
<%@page import="java.sql.Timestamp"%>

<%@page import="java.util.TreeMap"%>
<%
    String benutzerName = (String) session.getAttribute("benutzer");
    if (benutzerName == null) {
        response.sendRedirect("login.jsp");
        session.setAttribute("error", "0");
        return;
    }


    // TreeMap tm = DBManager.getInstance().getDataMap();
    // int intervall = DBManager.getInstance().getIntervall();
%>
<%@page contentType="text/html" pageEncoding="UTF-8"%>
<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//DE"
    "http://www.w3.org/TR/html4/loose.dtd">
<html>
    <head>
        <title>Messwerte</title>
        <!-- <meta http-equiv="refresh" content="100; URL=content.jsp">-->
        <link rel="stylesheet" type="text/css"
              href="html.css" media="all">
        <script type="text/javascript" src="js/jquery-1.4.4.min.js"></script>
        <script type="text/javascript" src="js/date.js"></script>
        <script type="text/javascript" src="js/jquery.datePicker.js"></script>


    </head>
    <body>
        <div id="wrappertop"></div>
        <div id="wrapper">

            <h1>SensorNetwork Controll Interface</h1>
            <h2>Logged in as <%=benutzerName%></h2>

            <% if (DBManager.getInstance().isAdmin(benutzerName)) {%> 
            <p>User is admin</p>
            <%}%>

            <form action="sendCommands.jsp" method="get">
                <input name="port" type="text" maxlength="255" value="<% out.print(SerialComm.getInstance().getPortName());%>"/> 
                <select name="baud"> 
                    <option value="<% out.print(SerialComm.getInstance().getBaud());%>"><% out.print(SerialComm.getInstance().getBaud());%></option>
                    <option value="9600" >9600</option>
                </select>
                <input type="submit" value="Connect RS232" name="connect"/>
            </form>

            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="get connected Nodes address" name="getnodeaddress"/>
            </form>

            <form action="sendCommands.jsp" method="get">

                <select name="node"> 
                    <option value="1" >1</option>
                    <option value="2" >2</option>
                    <option value="3" >3</option>
                </select>

                <input type="submit" value="Send getecho" name="getecho"/>

            </form>


            <form action="sendCommands.jsp" method="get">
                <select name="node"> 
                    <option value="1" >1</option>
                    <option value="2" >2</option>
                    <option value="3" >3</option>
                </select>
                <input type="submit" value="Send getdata" name="getdata"/>
            </form>
            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="Send getnodes" name="getnodes"/>
            </form>
            <form action="sendCommands.jsp" method="get">
                <input type="submit" value="Disconnect RS232" name="disconnect"/>
            </form>




            <form action="logout.jsp" method="get">

                <input type="submit" value="Logout" name="Logout" />
            </form>
        </div>
        <div id="wrapperbottom"></div>
    </body>
</html>
