using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class CameraCalibration
{
    public int device_index;
    public string serial_number;
    public bool is_valid;
    public float[][] rotation;      // 3x3 rotation matrix
    public float[] translation;     // 3x1 translation vector (mm)

    // Computed rotation matrix as flat array for faster access
    [NonSerialized]
    public float[] rotationFlat;

    public void ComputeFlatRotation()
    {
        if (rotation != null && rotation.Length == 3)
        {
            rotationFlat = new float[9];
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    rotationFlat[r * 3 + c] = rotation[r][c];
                }
            }
        }
    }
}

[Serializable]
public class CalibrationData
{
    public int num_devices;
    public CameraCalibration[] calibrations;
}

public static class CalibrationLoader
{
    public static CalibrationData LoadFromJson(string filePath)
    {
        if (!File.Exists(filePath))
        {
            Debug.LogError($"Calibration file not found: {filePath}");
            return null;
        }

        try
        {
            string jsonContent = File.ReadAllText(filePath);
            CalibrationData data = JsonUtility.FromJson<CalibrationData>(jsonContent);

            if (data == null || data.calibrations == null)
            {
                // JsonUtility doesn't handle nested arrays well, parse manually
                data = ParseCalibrationJson(jsonContent);
            }

            // Compute flat rotation matrices
            if (data != null && data.calibrations != null)
            {
                foreach (var cal in data.calibrations)
                {
                    cal.ComputeFlatRotation();
                }
            }

            Debug.Log($"Loaded calibration for {data?.num_devices} cameras");
            return data;
        }
        catch (Exception e)
        {
            Debug.LogError($"Failed to parse calibration file: {e.Message}");
            return null;
        }
    }

    private static CalibrationData ParseCalibrationJson(string json)
    {
        // Simple JSON parser for calibration format
        CalibrationData data = new CalibrationData();

        // Parse num_devices
        int numDevicesStart = json.IndexOf("\"num_devices\"") + 14;
        int numDevicesEnd = json.IndexOf(",", numDevicesStart);
        if (numDevicesEnd < 0) numDevicesEnd = json.IndexOf("}", numDevicesStart);
        string numDevicesStr = json.Substring(numDevicesStart, numDevicesEnd - numDevicesStart).Trim();
        data.num_devices = int.Parse(numDevicesStr);

        // Parse calibrations array
        List<CameraCalibration> calibrations = new List<CameraCalibration>();

        int calibrationsStart = json.IndexOf("\"calibrations\"");
        if (calibrationsStart < 0) return data;

        int arrayStart = json.IndexOf("[", calibrationsStart);
        int arrayEnd = FindMatchingBracket(json, arrayStart, '[', ']');

        string calibrationsContent = json.Substring(arrayStart + 1, arrayEnd - arrayStart - 1);

        // Find each calibration object
        int searchStart = 0;
        while (true)
        {
            int objStart = calibrationsContent.IndexOf("{", searchStart);
            if (objStart < 0) break;

            int objEnd = FindMatchingBracket(calibrationsContent, objStart, '{', '}');
            string objContent = calibrationsContent.Substring(objStart, objEnd - objStart + 1);

            CameraCalibration cal = ParseCameraCalibration(objContent);
            if (cal != null)
            {
                calibrations.Add(cal);
            }

            searchStart = objEnd + 1;
        }

        data.calibrations = calibrations.ToArray();
        return data;
    }

    private static CameraCalibration ParseCameraCalibration(string json)
    {
        CameraCalibration cal = new CameraCalibration();

        // device_index
        cal.device_index = ParseInt(json, "device_index");

        // serial_number
        cal.serial_number = ParseString(json, "serial_number");

        // is_valid
        cal.is_valid = ParseBool(json, "is_valid");

        if (cal.is_valid)
        {
            // rotation (3x3 matrix)
            cal.rotation = ParseMatrix3x3(json, "rotation");

            // translation (3x1 vector)
            cal.translation = ParseVector3(json, "translation");
        }
        else
        {
            // Identity for primary camera
            cal.rotation = new float[][] {
                new float[] { 1, 0, 0 },
                new float[] { 0, 1, 0 },
                new float[] { 0, 0, 1 }
            };
            cal.translation = new float[] { 0, 0, 0 };
        }

        cal.ComputeFlatRotation();
        return cal;
    }

    private static int ParseInt(string json, string key)
    {
        int keyStart = json.IndexOf($"\"{key}\"");
        if (keyStart < 0) return 0;

        int valueStart = json.IndexOf(":", keyStart) + 1;
        int valueEnd = json.IndexOfAny(new char[] { ',', '}', '\n' }, valueStart);
        string valueStr = json.Substring(valueStart, valueEnd - valueStart).Trim();
        return int.Parse(valueStr);
    }

    private static string ParseString(string json, string key)
    {
        int keyStart = json.IndexOf($"\"{key}\"");
        if (keyStart < 0) return "";

        int valueStart = json.IndexOf("\"", json.IndexOf(":", keyStart) + 1) + 1;
        int valueEnd = json.IndexOf("\"", valueStart);
        return json.Substring(valueStart, valueEnd - valueStart);
    }

    private static bool ParseBool(string json, string key)
    {
        int keyStart = json.IndexOf($"\"{key}\"");
        if (keyStart < 0) return false;

        int valueStart = json.IndexOf(":", keyStart) + 1;
        int valueEnd = json.IndexOfAny(new char[] { ',', '}', '\n' }, valueStart);
        string valueStr = json.Substring(valueStart, valueEnd - valueStart).Trim().ToLower();
        return valueStr == "true";
    }

    private static float[][] ParseMatrix3x3(string json, string key)
    {
        int keyStart = json.IndexOf($"\"{key}\"");
        if (keyStart < 0) return null;

        int arrayStart = json.IndexOf("[", keyStart);
        int arrayEnd = FindMatchingBracket(json, arrayStart, '[', ']');
        string matrixContent = json.Substring(arrayStart + 1, arrayEnd - arrayStart - 1);

        float[][] matrix = new float[3][];
        int rowIndex = 0;
        int searchStart = 0;

        while (rowIndex < 3)
        {
            int rowStart = matrixContent.IndexOf("[", searchStart);
            if (rowStart < 0) break;

            int rowEnd = matrixContent.IndexOf("]", rowStart);
            string rowContent = matrixContent.Substring(rowStart + 1, rowEnd - rowStart - 1);

            string[] values = rowContent.Split(',');
            matrix[rowIndex] = new float[3];
            for (int c = 0; c < 3 && c < values.Length; c++)
            {
                matrix[rowIndex][c] = float.Parse(values[c].Trim(), System.Globalization.CultureInfo.InvariantCulture);
            }

            rowIndex++;
            searchStart = rowEnd + 1;
        }

        return matrix;
    }

    private static float[] ParseVector3(string json, string key)
    {
        int keyStart = json.IndexOf($"\"{key}\"");
        if (keyStart < 0) return new float[] { 0, 0, 0 };

        int arrayStart = json.IndexOf("[", keyStart);
        int arrayEnd = json.IndexOf("]", arrayStart);
        string arrayContent = json.Substring(arrayStart + 1, arrayEnd - arrayStart - 1);

        string[] values = arrayContent.Split(',');
        float[] vector = new float[3];
        for (int i = 0; i < 3 && i < values.Length; i++)
        {
            vector[i] = float.Parse(values[i].Trim(), System.Globalization.CultureInfo.InvariantCulture);
        }

        return vector;
    }

    private static int FindMatchingBracket(string str, int start, char open, char close)
    {
        int depth = 0;
        for (int i = start; i < str.Length; i++)
        {
            if (str[i] == open) depth++;
            else if (str[i] == close) depth--;

            if (depth == 0) return i;
        }
        return str.Length - 1;
    }
}
