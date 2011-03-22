/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package jdbc;

import com.mysql.jdbc.exceptions.MySQLIntegrityConstraintViolationException;
import java.sql.*;
import java.util.TreeMap;
import java.util.logging.Level;
import java.util.logging.Logger;

public class DBManager {

    private static final DBManager instance = new DBManager();
    private static Driver jdbcDriver;
    private static String userName = "simon";
    private static String passwd = "123";
    private static String url = "jdbc:mysql://localhost:3306/login";
    private Connection con;
    private Statement stm;
    private ResultSet rs;

    static {
        try {
            jdbcDriver = (Driver) Class.forName("com.mysql.jdbc.Driver").newInstance();
            DriverManager.registerDriver(jdbcDriver);
        } catch (Exception ex) {
            ex.printStackTrace();
        }

    }
    private static final int STDVALUE = 5;

    private DBManager() {
    }

    public static DBManager getInstance() {
        return instance;
    }

    public java.sql.Connection getConnection() throws SQLException {
        if (con == null) {
            con = DriverManager.getConnection(url, userName, passwd);
        }
        return con;
    }

    public boolean createNewUser(String username, String password, int admin) {
        try {
            this.getConnection();
            stm = con.createStatement();
        } catch (MySQLIntegrityConstraintViolationException ex) {
            System.err.println("User allready exists");
            return false;
        } catch (SQLException ex) {
            ex.printStackTrace();
            return false;
        }

//        rs = stm.executeQuery("SELECT * FROM mitarbeiter m");
//        ArrayList<String> namelst = new ArrayList<String>();
//        while (rs.next()){
//            namelst.add(rs.getString("vorname"));
//        }
        String sql = "";
        if ((!username.isEmpty()) && (!password.isEmpty())) {
            System.out.println("create user in db");
            sql = "INSERT INTO logindata (username, password, admin) VALUES ('" + username + "', '" + password + "','" + admin + "');";
            try {
                stm.executeUpdate(sql);
                return true;
            } catch (SQLException ex) {
                Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex);
                return false;
            }
        } else {
            System.err.println("kein Username oder Password angegeben");
            return false;
        }

    }

    public boolean validiereUser(String username, String password) throws SQLException {
        this.getConnection();
        stm = con.createStatement();
        String sql = "Select * from logindata where username='" + username + "' and password='" + password + "'";
        try {
            ResultSet r = stm.executeQuery(sql);

            if (r.next()) {
                stm.close();
                return true;
            }
            stm.close();
            return false;
        } catch (SQLException ex) {
            Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex);
            System.err.println("sql exception in validiere user");
            return false;
        }
    }

    public boolean isAdmin(String username) throws SQLException {
        this.getConnection();
        stm = con.createStatement();
        String sql = "Select * from logindata where username='" + username + "' and admin = '1';";

        try {
            ResultSet r = stm.executeQuery(sql);

            if (r.next()) {
                System.out.println("user is admin");
                return true;
            }
            System.out.println("user is NOT admin");
            return false;
        } catch (SQLException ex) {
            Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println("sql exception in isAdmin");
            return false;
        }
    }

    //method called to get full data to the client
    public TreeMap<Timestamp, Integer> getDataMap() {
        TreeMap tm = new TreeMap<Timestamp, Integer>();
        int data;
        Timestamp ts;
        try {
            this.getConnection();
            stm = con.createStatement();
//            String sql = "SELECT * FROM messdata where timestamp < from_unixtime('" + to.getTime() / 1000
//                    + "') and timestamp > from_unixtime('" + from.getTime() / 1000 + "');";
            String sql = "SELECT * FROM messdata order by id desc ;";
            rs = stm.executeQuery(sql);

            while (rs.next()) {
                ts = rs.getTimestamp("timestamp");
                data = rs.getInt("data");
                tm.put(ts, data);
            }

            return tm;
        } catch (SQLException ex) {
            System.err.println("Fehler in getdatamap");
            return tm;
        }
    }

    public boolean setIntervall(int intervall, String username) {
        try {
            stm = con.createStatement();
            String sql = "";
            sql = "INSERT INTO intervall VALUES (from_unixtime(" + System.currentTimeMillis() + "),'" + intervall + "', '" + username + "');";
            stm.executeUpdate(sql);
            stm.close();

            return true;
        } catch (SQLException ex) {
            Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex);
            return false;
        }
    }

    public int getIntervall() {
        String sql = "";
        sql = "select * from intervall order by timestamp desc limit 1";
        try {
            stm = con.createStatement();
            rs = stm.executeQuery(sql);
            rs.next();
            System.out.println(rs.getInt("intervall"));
            return rs.getInt("intervall");

        } catch (SQLException ex) {
            Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex);
            try {
                stm.close();
            } catch (SQLException ex1) {
                Logger.getLogger(DBManager.class.getName()).log(Level.SEVERE, null, ex1);
            }
            return STDVALUE;

        }
    }
}
