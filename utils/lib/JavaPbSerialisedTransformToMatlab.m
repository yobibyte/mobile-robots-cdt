function [varargout] = JavaPbSerialisedTransformToMatlab(varargin)

%
% JavaPbSerialisedTransformToMatlab - decode a java protobuf
%      pbSerialisedTransform into a MATLAB SE(3) transform.
%
% [ SE3, source_frame, destination_frame, source_timestamp, XYZRPY,
%   se3_euler_covariance, destination_timestamp ] =
%      JavaPbSerialisedTransformToMatlab( pbSerialisedTransform )
%
% INPUTS:
%   pbSerialisedTransform: Java object of class mrg.datatypes.protobuf.
%     transform.PbSerialisedTransform.
%
% OUTPUTS:
%   SE3: 4-by-4 homogeneous SE(3) Yaw-Pitch-Roll representation of transform
%     encoded in pbSerialisedTransform.
%   source_frame: String containing the SOURCE frame for the SE3 transform,
%     ie: (ie: p_destination = G_destination_source * p_source).
%   destination_frame: String containing the DESTINATION frame for the SE3
%     transform, ie: (ie: p_destination = G_destination_source * p_source).
%   source_timestamp: UNIX int64 type defining the source frame timestamp
%     of the transform.
%   XYZRPY: linear and angular components corresponding to 4-by-4 SE(3)
%   destination_timestamp: UNIX int64 type defining the destination frame
%     timestamp of the transform.
%
%  LEAF TRANSFORM ONLY:
%   se3_euler_covariance: 6-by-6 numeric matrix defining the covariance
%      of the SE(3) transform using the Euler angle parameterisation,
%      obtained by transforming from the se3 Lie Algebra parameterisation.
%      Currently extracting this is only supported for Leaf transforms in MATLAB.
%
% NOTES:
%   If pbSerialisedTransform is a composite, then the result is flattened to
%   an equivalent leaf transform for output.
%
% SEE ALSO:
%   MatlabToJavaPbSerialisedTransform
%

% Will Maddern
% April 2013
% Mobile Robotics Group, Oxford University.

% parse inputs
arg_index = 1;

if ( ~( isjava(varargin{arg_index}) && ...
        strfind(char(varargin{arg_index}.getClass().toString()), ...
                'mrg.datatypes.protobuf.transform.PbSerialisedTransform') ) )
  error(['%s - pbSerialisedTransform input must be a java protobuf class of ' ...
         'type: mrg.datatypes.protobuf.transform.PbSerialisedTransform'], ...
        mfilename);
end
pb_serialised_transform = varargin{arg_index};
arg_index = arg_index+1;

se3 = eye(4);
source_frame = '';
destination_frame = '';
xyzrpy = nan(1,6);
% Default covariance assumes perfectly known (no uncertainty).
se3_euler_covariance = zeros(6);

switch char(pb_serialised_transform.getTransformType.toString)
  case 'TRANSFORM'
    pb_transform = ...
        javaMethod('parseFrom', ...
                   'mrg.datatypes.protobuf.transform.PbTransform$pbTransform', ...
                   typecast(pb_serialised_transform.getSerialisedTransform ...
                            .toByteArray,'uint8'));
    source_frame = char(pb_transform.getSourceFrame());
    destination_frame = char(pb_transform.getDestinationFrame());
    source_timestamp = pb_transform.getSourceTimestamp();
    if (pb_transform.hasDestinationTimestamp())
      destination_timestamp = pb_transform.getDestinationTimestamp();
    else
      % Legacy data had only a single (source) timestamp.
      destination_timestamp = source_timestamp;
    end
    xyzrpy = [reshape(cell2mat(cell(pb_transform.getLinXyzList.toArray)),1,3), ...
              reshape(cell2mat(cell(pb_transform.getAngRpyList.toArray)),1,3)];
    se3 = BuildSE3Transform(xyzrpy);
    if (pb_transform.getSe3LieAlgebraCovarianceCount() > 0)
      se3_lie_algebra_covariance = ...
          reshape(cell2mat(cell(...
              pb_transform.getSe3LieAlgebraCovarianceList.toArray)), 6, 6);
      % Convert to Euler covariance from se3 Lie Algebra for MATLAB.
      se3_euler_covariance = ...
          LieAlgebraCovarianceToEulerCovariance(...
              se3, se3_lie_algebra_covariance);
    end
  case 'COMPOSITE_TRANSFORM'
    pb_composite_transform = ...
        javaMethod('parseFrom', ...
                   ['mrg.datatypes.protobuf.transform.PbCompositeTransform', ...
                    '$pbCompositeTransform'], ...
                   typecast(pb_serialised_transform.getSerialisedTransform ...
                            .toByteArray,'uint8'));
    source_timestamp = pb_composite_transform.getSourceTimestamp();

    if (pb_composite_transform.hasDestinationTimestamp())
      destination_timestamp = ...
          pb_composite_transform.getDestinationTimestamp();
    else
      % Legacy data had only a single (source) timestamp.
      destination_timestamp = source_timestamp;
    end

    % flatten transform tree into a single transform, note that ordering is
    % _written_ order, thus (1) is the _last_ transform to be applied, (end)
    % the first in any operation
    for k=0:(pb_composite_transform.getTransformChainEntriesCount-1)
      [child_se3, child_source, child_destination] = ...
          JavaPbSerialisedTransformToMatlab(...
              pb_composite_transform.getTransformChainEntries(k));
      se3 = se3 * child_se3;
      if ( k == 0 ) destination_frame = child_destination; end
    end
    source_frame = child_source;
    xyzrpy = SE3ToComponents(se3);

    % TODO(alex): Currently outputting covariances is only supported for
    %             leaf transform as we would have to replicate the
    %             combination of covariances in MATLAB.
end

% prepare outputs
out_idx = 1;
if ( nargout >= out_idx )
  varargout(out_idx) = { se3 };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { source_frame };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { destination_frame };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { source_timestamp };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { xyzrpy };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { se3_euler_covariance };
  out_idx = out_idx+1;
end
if ( nargout >= out_idx )
  varargout(out_idx) = { destination_timestamp };
  out_idx = out_idx+1;
end


% --------------------------------------------------------------------------
function [se3] = BuildSE3Transform(xyzrpy)

R_x = [ 1, 0, 0;
        0, cos(xyzrpy(4)), -sin(xyzrpy(4));
        0, sin(xyzrpy(4)), cos(xyzrpy(4)) ];

R_y = [ cos(xyzrpy(5)), 0, sin(xyzrpy(5));
        0, 1, 0;
        -sin(xyzrpy(5)), 0, cos(xyzrpy(5)) ];

R_z = [ cos(xyzrpy(6)), -sin(xyzrpy(6)), 0;
        sin(xyzrpy(6)), cos(xyzrpy(6)), 0;
        0, 0, 1 ];

% Yaw Pitch Roll (YPR) Euler angle order
R_zyx = R_z * R_y * R_x;
se3 = nan(4);
se3(1:3,1:3) = R_zyx;
se3(1:3,4) = xyzrpy(1:3);
se3(4,:) = [zeros(1,3), 1];
