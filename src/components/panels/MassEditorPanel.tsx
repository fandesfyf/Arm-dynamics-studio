import { useCallback, useEffect, useMemo, useState } from 'react';
import {
  MassEditor,
  type JointLimitInfo,
  type LinkInertial,
} from '../../core/mass-editor';

export interface MassEditorPanelProps {
  urdfXml: string;
  onUrdfChanged: (xml: string) => void;
}

type LinkDraft = LinkInertial;
type JointDraft = JointLimitInfo;

export function MassEditorPanel({ urdfXml, onUrdfChanged }: MassEditorPanelProps) {
  const parsed = useMemo(() => {
    const editor = new MassEditor(urdfXml);
    return {
      links: editor.getLinkInertials(),
      joints: editor.getJointLimits(),
    };
  }, [urdfXml]);

  const [links, setLinks] = useState<LinkDraft[]>(parsed.links);
  const [joints, setJoints] = useState<JointDraft[]>(parsed.joints);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setLinks(parsed.links);
    setJoints(parsed.joints);
    setError(null);
  }, [parsed]);

  const updateLink = useCallback(
    (index: number, patch: Partial<LinkDraft>) => {
      setLinks((prev) =>
        prev.map((row, i) => (i === index ? { ...row, ...patch } : row)),
      );
    },
    [],
  );

  const updateLinkInertia = useCallback(
    (index: number, key: keyof LinkDraft['inertia'], value: number) => {
      setLinks((prev) =>
        prev.map((row, i) =>
          i === index
            ? { ...row, inertia: { ...row.inertia, [key]: value } }
            : row,
        ),
      );
    },
    [],
  );

  const updateJoint = useCallback(
    (index: number, patch: Partial<JointDraft>) => {
      setJoints((prev) =>
        prev.map((row, i) => (i === index ? { ...row, ...patch } : row)),
      );
    },
    [],
  );

  const handleApply = useCallback(() => {
    try {
      const editor = new MassEditor(urdfXml);
      for (const link of links) {
        editor.setLinkMass(link.linkName, link.mass);
        editor.setLinkInertia(
          link.linkName,
          link.inertia.ixx,
          link.inertia.iyy,
          link.inertia.izz,
        );
      }
      for (const joint of joints) {
        editor.setJointLimits(joint.jointName, {
          lower: joint.lower,
          upper: joint.upper,
          effort: joint.effort,
          velocity: joint.velocity,
        });
      }
      setError(null);
      onUrdfChanged(editor.serialize());
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    }
  }, [urdfXml, links, joints, onUrdfChanged]);

  return (
    <section className="mass-editor-panel">
      <h3>惯量 / 关节限位</h3>
      {error && <p className="mass-editor-error">{error}</p>}

      <h4>Link 质量与惯量</h4>
      <div className="mass-editor-table-wrap">
        <table className="mass-editor-table">
          <thead>
            <tr>
              <th>Link</th>
              <th>质量 (kg)</th>
              <th>ixx</th>
              <th>iyy</th>
              <th>izz</th>
            </tr>
          </thead>
          <tbody>
            {links.map((link, index) => (
              <tr key={link.linkName}>
                <td>{link.linkName}</td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={link.mass}
                    onChange={(e) =>
                      updateLink(index, { mass: Number(e.target.value) })
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={link.inertia.ixx}
                    onChange={(e) =>
                      updateLinkInertia(index, 'ixx', Number(e.target.value))
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={link.inertia.iyy}
                    onChange={(e) =>
                      updateLinkInertia(index, 'iyy', Number(e.target.value))
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={link.inertia.izz}
                    onChange={(e) =>
                      updateLinkInertia(index, 'izz', Number(e.target.value))
                    }
                  />
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <h4>关节限位</h4>
      <div className="mass-editor-table-wrap">
        <table className="mass-editor-table">
          <thead>
            <tr>
              <th>Joint</th>
              <th>lower</th>
              <th>upper</th>
              <th>effort</th>
              <th>velocity</th>
            </tr>
          </thead>
          <tbody>
            {joints.map((joint, index) => (
              <tr key={joint.jointName}>
                <td>{joint.jointName}</td>
                <td>
                  <input
                    type="number"
                    step="any"
                    value={joint.lower}
                    onChange={(e) =>
                      updateJoint(index, { lower: Number(e.target.value) })
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    step="any"
                    value={joint.upper}
                    onChange={(e) =>
                      updateJoint(index, { upper: Number(e.target.value) })
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={joint.effort}
                    onChange={(e) =>
                      updateJoint(index, { effort: Number(e.target.value) })
                    }
                  />
                </td>
                <td>
                  <input
                    type="number"
                    min={0}
                    step="any"
                    value={joint.velocity}
                    onChange={(e) =>
                      updateJoint(index, { velocity: Number(e.target.value) })
                    }
                  />
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>

      <button type="button" className="primary" onClick={handleApply}>
        应用修改
      </button>
    </section>
  );
}
